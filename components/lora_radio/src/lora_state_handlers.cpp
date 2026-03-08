#include "lora_radio.hpp"

#include "sdkconfig.h"
#if CONFIG_LORA_LOG_LEVEL_NONE
#define LOG_LOCAL_LEVEL ESP_LOG_NONE
#elif CONFIG_LORA_LOG_LEVEL_ERROR
#define LOG_LOCAL_LEVEL ESP_LOG_ERROR
#elif CONFIG_LORA_LOG_LEVEL_WARN
#define LOG_LOCAL_LEVEL ESP_LOG_WARN
#elif CONFIG_LORA_LOG_LEVEL_INFO
#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#elif CONFIG_LORA_LOG_LEVEL_DEBUG
#define LOG_LOCAL_LEVEL ESP_LOG_DEBUG
#else
#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#endif
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include <cstdlib>

static const char* TAG = "LoraRadio";

const LoraRadio::StateHandler LoraRadio::s_handlers[STATE_HANDLER_COUNT] = {
    &LoraRadio::handleIdle,
    &LoraRadio::handleCad,
    &LoraRadio::handleTxWait,
    &LoraRadio::handleWaitAck,
    &LoraRadio::handleRx,
    &LoraRadio::handleSleeping,
};

void LoraRadio::dispatchRx(const uint8_t* frame, size_t len, float rssi, float snr)
{
    if (len < PACKET_HEADER_SIZE) {
        ESP_LOGD(TAG, "RX: frame too short (%zu bytes)", len);
        return;
    }

    const PacketHeader* hdr = reinterpret_cast<const PacketHeader*>(frame);

    ESP_LOGD(TAG, "RX: src=0x%04X dst=0x%04X seq=%u type=%u flags=0x%02X len=%u RSSI=%.1f SNR=%.1f",
             hdr->srcId,
             hdr->dstId,
             hdr->seqNum,
             hdr->msgType,
             hdr->flags,
             hdr->payloadLen,
             static_cast<double>(rssi),
             static_cast<double>(snr));

    _neighbors.update(hdr->srcId, rssi, snr);

    if (hdr->dstId != 0xFFFF && hdr->dstId != _cfg.nodeId) {
        return;
    }

    if (hdr->msgType == MSGTYPE_ACK) {
        return;
    }

    if (hdr->flags & FLAG_ACK_REQUEST) {
        TxItem ackItem{};
        buildAckFrame(ackItem.buf, ackItem.len, *hdr, _cfg.nodeId);
        ackItem.needsAck = false;
        if (xQueueSend(_normalQueue, &ackItem, 0) != pdTRUE) {
            ESP_LOGW(TAG, "dispatchRx: TX queue full, ACK dropped");
        }
    }

    if (_rxCb) {
        const uint8_t* payload = frame + PACKET_HEADER_SIZE;
        _rxCb(*hdr, payload, rssi, snr);
    }
}

bool LoraRadio::handleIdle()
{
    uint32_t bits = 0;
    xTaskNotifyWait(0, NOTIFY_STOP, &bits, 0);
    if (bits & NOTIFY_STOP) {
        ESP_LOGI(TAG, "STOP received - exiting task loop");
        return false;
    }

    if (dequeueNextTx(_currentTx)) {
        _hasPendingTx = true;
        _cadRetries = 0;
        ESP_LOGD(TAG, "IDLE->CAD: queued %u bytes", _currentTx.len);
        int16_t cadErr = _backend->startChannelScan();
        if (cadErr != RADIO_ERR_NONE) {
            ESP_LOGW(TAG, "startChannelScan failed (%d), skipping CAD", cadErr);
            _state = State::TX_WAIT;
            _backend->startTransmit(_currentTx.buf, _currentTx.len);
        } else {
            _state = State::CAD;
        }
    } else {
        _backend->startReceive();
        _state = State::RX;
    }

    return true;
}

bool LoraRadio::handleCad()
{
    uint32_t bits = waitNotify(500);

    if (bits & NOTIFY_STOP) {
        return false;
    }

    if (bits & NOTIFY_DIO1) {
        ESP_LOGD(TAG, "CAD: channel busy (retry %u/%u)",
                 _cadRetries + 1,
                 static_cast<unsigned>(_cfg.cadRetries));

        if (++_cadRetries < _cfg.cadRetries) {
            uint32_t backoffMs = 50 + static_cast<uint32_t>(rand() % 150);
            vTaskDelay(pdMS_TO_TICKS(backoffMs));
            _backend->startChannelScan();
        } else {
            ESP_LOGW(TAG, "CAD: max retries reached, dropping packet");
            _hasPendingTx = false;
            _state = State::IDLE;
        }
    } else if (bits & NOTIFY_DIO0) {
        int16_t result = _backend->getChannelScanResult();
        if (result == RADIO_LORA_DETECTED) {
            ESP_LOGD(TAG, "CAD: LORA_DETECTED on DIO0, retrying");
            if (++_cadRetries < _cfg.cadRetries) {
                uint32_t backoffMs = 50 + static_cast<uint32_t>(rand() % 150);
                vTaskDelay(pdMS_TO_TICKS(backoffMs));
                _backend->startChannelScan();
            } else {
                ESP_LOGW(TAG, "CAD: max retries, dropping");
                _hasPendingTx = false;
                _state = State::IDLE;
            }
        } else {
            ESP_LOGD(TAG, "CAD: channel clear -> TX");
            _backend->startTransmit(_currentTx.buf, _currentTx.len);
            _state = State::TX_WAIT;
        }
    } else {
        ESP_LOGD(TAG, "CAD: timeout, assuming clear -> TX");
        _backend->startTransmit(_currentTx.buf, _currentTx.len);
        _state = State::TX_WAIT;
    }

    return true;
}

bool LoraRadio::handleTxWait()
{
    uint32_t bits = waitNotify(5000);

    if (bits & NOTIFY_STOP) {
        return false;
    }

    if (bits & NOTIFY_DIO0) {
        _backend->finishTransmit();
        ESP_LOGD(TAG, "TX done: %u bytes", _currentTx.len);

        if (_currentTx.needsAck) {
            _ackTracker.startTracking(_currentTx);
            _backend->startReceive();
            _state = State::WAIT_ACK;
        } else {
            _hasPendingTx = false;
            _state = State::IDLE;
        }
    } else {
        ESP_LOGW(TAG, "TX_WAIT timeout - resetting");
        _backend->standby();
        _hasPendingTx = false;
        _state = State::IDLE;
    }

    return true;
}

bool LoraRadio::handleWaitAck()
{
    uint32_t bits = waitNotify(_cfg.ackTimeoutMs);

    if (bits & NOTIFY_STOP) {
        return false;
    }

    if (bits & NOTIFY_DIO0) {
        size_t pktLen = _backend->getPacketLength();
        uint8_t buf[MAX_FRAME_SIZE] = {};
        if (pktLen > 0 && pktLen <= MAX_FRAME_SIZE) {
            _backend->readData(buf, pktLen);
            float rssi = _backend->getRSSI();
            float snr = _backend->getSNR();

            if (pktLen >= PACKET_HEADER_SIZE) {
                const PacketHeader* rxHdr = reinterpret_cast<const PacketHeader*>(buf);

                if (_ackTracker.matchesAck(*rxHdr, _cfg.nodeId)) {
                    ESP_LOGD(TAG, "ACK received for seq=%u", rxHdr->seqNum);
                    _neighbors.update(rxHdr->srcId, rssi, snr);
                    _ackTracker.clear();
                    _hasPendingTx = false;
                    if (_ackCb) {
                        _ackCb(rxHdr->seqNum, true);
                    }
                    _state = State::IDLE;
                } else {
                    dispatchRx(buf, pktLen, rssi, snr);
                    _backend->startReceive();
                }
            }
        }
    } else {
        if (_ackTracker.shouldRetry(_cfg.maxRetries)) {
            ESP_LOGD(TAG, "ACK timeout: retransmit %u/%lu",
                     _ackTracker.pending().retries,
                     _cfg.maxRetries);
            _currentTx = _ackTracker.pending();
            _cadRetries = 0;
            _backend->startChannelScan();
            _state = State::CAD;
        } else {
            ESP_LOGW(TAG, "ACK: max retries (%lu) exhausted, dropping", _cfg.maxRetries);
            uint8_t failedSeq = _ackTracker.failedSeqNum();
            _ackTracker.clear();
            _hasPendingTx = false;
            if (_ackCb) {
                _ackCb(failedSeq, false);
            }
            _state = State::IDLE;
        }
    }

    return true;
}

bool LoraRadio::handleRx()
{
    uint32_t bits = waitNotify(_cfg.sleepIdleMs);

    if (bits & NOTIFY_STOP) {
        return false;
    }
    if (bits & NOTIFY_TX_QUEUED) {
        _backend->standby();
        _state = State::IDLE;
        return true;
    }

    if (bits & NOTIFY_DIO0) {
        size_t pktLen = _backend->getPacketLength();
        uint8_t buf[MAX_FRAME_SIZE] = {};
        if (pktLen > 0 && pktLen <= MAX_FRAME_SIZE) {
            int16_t rxErr = _backend->readData(buf, pktLen);
            if (rxErr == RADIO_ERR_NONE) {
                float rssi = _backend->getRSSI();
                float snr = _backend->getSNR();
                dispatchRx(buf, pktLen, rssi, snr);
            } else {
                ESP_LOGD(TAG, "readData error %d", rxErr);
            }
        }
        _state = State::IDLE;
    } else {
        ESP_LOGD(TAG, "Idle timeout -> SLEEPING");
        _backend->sleep();
        _state = State::SLEEPING;
    }

    return true;
}

bool LoraRadio::handleSleeping()
{
    ESP_LOGI(TAG, "Modem sleeping (idle >%lu ms)", _cfg.sleepIdleMs);

    uint32_t bits = waitNotify(portMAX_DELAY);

    if (bits & NOTIFY_STOP) {
        return false;
    }

    if (bits & (NOTIFY_TX_QUEUED | NOTIFY_DIO0)) {
        _backend->standby();
        ESP_LOGI(TAG, "Modem wake-up");
        _state = State::IDLE;
    }

    return true;
}
