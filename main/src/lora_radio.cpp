// ---------------------------------------------------------------------------
// lora_radio.cpp — LoRa V2V radio state machine + FreeRTOS task
// ---------------------------------------------------------------------------
// Implementation notes:
//
//  • RadioLib objects (EspHal, Module, SX1278) are heap-allocated exactly
//    once inside init() and freed inside deinit().  No heap allocation
//    occurs inside the operational loops.
//
//  • All FreeRTOS objects (task, queues) use static storage declared below.
//    xTaskCreateStatic / xQueueCreateStatic never call pvPortMalloc.
//
//  • DIO0 and DIO1 ISRs contain only xTaskNotifyFromISR + portYIELD_FROM_ISR.
//    Zero SPI transactions, zero blocking calls, zero logic in ISR context.
//
//  • The HAL is injected into the SX1278 via the Module constructor:
//      Module mod(hal, nss, dio0, rst, …);
//    This is the correct RadioLib non-Arduino pattern — the radio object
//    never touches hardware directly; all I/O is routed through the HAL.
// ---------------------------------------------------------------------------

// RadioLib must be included before our own headers so its macros are visible
#include <RadioLib.h>

#include "lora_radio.hpp"
#include "esp_hal_s3.hpp"

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include <cstring>
#include <cstdlib>   // rand()
#include <algorithm>

static const char* TAG = "LoraRadio";

// Link-layer ACK message type — internal to this translation unit.
// Callers must not use 0x04 as their own msgType.
static constexpr uint8_t MSGTYPE_ACK = 0x04;

// ===========================================================================
// Static storage — allocated in BSS section, never on the heap after init()
// ===========================================================================

// Task stack and TCB
StackType_t  LoraRadio::s_taskStack[CONFIG_LORA_TASK_STACK_SIZE];
StaticTask_t LoraRadio::s_taskTcb;

// TX queue
uint8_t       LoraRadio::s_normalQueueStorage[
    CONFIG_LORA_TX_QUEUE_DEPTH * sizeof(LoraRadio::TxItem)];
StaticQueue_t LoraRadio::s_normalQueueState;

// Neighbour table
NeighborEntry LoraRadio::s_neighbors[CONFIG_LORA_MAX_NEIGHBORS];

// Singleton pointer — written in init(), used only by ISR trampolines
LoraRadio*    LoraRadio::s_instance = nullptr;

// ===========================================================================
// Convenience cast helpers (keep RadioLib headers out of lora_radio.hpp)
// ===========================================================================

static inline Module*  toModule(void* p) { return reinterpret_cast<Module*>(p);  }
static inline SX1278*  toRadio(void* p)  { return reinterpret_cast<SX1278*>(p);  }

// ===========================================================================
// Constructor / Destructor
// ===========================================================================

LoraRadio::LoraRadio(const LoraRadioConfig& cfg)
    : _cfg(cfg)
{
    std::memset(s_neighbors, 0, sizeof(s_neighbors));
}

LoraRadio::~LoraRadio()
{
    deinit();
}

// ===========================================================================
// Lifecycle
// ===========================================================================

esp_err_t LoraRadio::init()
{
    if (_initialized) {
        ESP_LOGW(TAG, "init() called twice — ignoring");
        return ESP_OK;
    }

    // ------------------------------------------------------------------
    // 1. Construct and initialise the ESP-IDF SPI HAL.
    //    The HAL object is the concrete implementation of RadioLibHal.
    //    It is injected into the Module, which is injected into SX1278.
    //    This is the RadioLib non-Arduino HAL-injection pattern.
    // ------------------------------------------------------------------
    _hal = new EspHal(
        static_cast<spi_host_device_t>(_cfg.spiHost),
        _cfg.pinSck,
        _cfg.pinMiso,
        _cfg.pinMosi
    );
    _hal->init();   // → spi_bus_initialize + spi_bus_add_device

    // ------------------------------------------------------------------
    // 2. Build the RadioLib Module.
    //    Module(hal, cs, irq, rst, gpio)
    //    cs   = NSS pin (RadioLib toggles it via hal->digitalWrite)
    //    irq  = DIO0 — primary interrupt (TX/RX done, CAD no-activity)
    //    rst  = hardware reset pin
    //    gpio = RADIOLIB_NC (we manage DIO1 separately via setDio1Action)
    // ------------------------------------------------------------------
    _module = new Module(
        _hal,
        _cfg.pinNss,
        _cfg.pinDio0,
        _cfg.pinRst,
        RADIOLIB_NC      // DIO1 attached manually below
    );

    // ------------------------------------------------------------------
    // 3. Construct SX1278 and configure RF parameters.
    //    begin() performs the hardware reset, reads the chip version,
    //    and writes all the modem registers.
    // ------------------------------------------------------------------
    SX1278* radio = new SX1278(toModule(_module));
    _radio = radio;

    ESP_LOGI(TAG, "Initialising SX1278 @ %.3f MHz  SF%u  BW%.0fkHz  CR4/%u  +%ddBm",
             (double)_cfg.frequencyMHz,
             _cfg.spreadingFactor,
             (double)_cfg.bandwidthKHz,
             _cfg.codingRate,
             _cfg.txPowerDbm);

    int16_t state = radio->begin(
        _cfg.frequencyMHz,
        _cfg.bandwidthKHz,
        _cfg.spreadingFactor,
        _cfg.codingRate,
        _cfg.syncWord,
        _cfg.txPowerDbm
    );

    if (state != RADIOLIB_ERR_NONE) {
        ESP_LOGE(TAG, "SX1278::begin() failed, code %d", state);
        delete radio; _radio = nullptr;
        delete toModule(_module); _module = nullptr;
        delete _hal; _hal = nullptr;
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "SX1278 init OK");

    // ------------------------------------------------------------------
    // 4. Attach the ISR trampolines via RadioLib.
    //    setDio0Action / setDio1Action call hal->attachInterrupt internally.
    //    Our trampolines contain ONLY xTaskNotifyFromISR — no SPI, no logic.
    // ------------------------------------------------------------------
    radio->setDio0Action(LoraRadio::dio0Isr, RADIOLIB_HAL_RISING);

    if (_cfg.pinDio1 != RADIOLIB_NC) {
        // DIO1 fires when CAD detects a LoRa preamble (channel busy).
        radio->setDio1Action(LoraRadio::dio1Isr, RADIOLIB_HAL_RISING);
    } else {
        ESP_LOGW(TAG, "DIO1 not connected — CAD busy detection disabled");
    }

    // ------------------------------------------------------------------
    // 5. Create statically-allocated FreeRTOS TX queue.
    //    xQueueCreateStatic never calls pvPortMalloc.
    // ------------------------------------------------------------------
    _normalQueue = xQueueCreateStatic(
        CONFIG_LORA_TX_QUEUE_DEPTH,
        sizeof(TxItem),
        s_normalQueueStorage,
        &s_normalQueueState
    );

    // ------------------------------------------------------------------
    // 6. Set singleton pointer before creating the task.
    //    The ISR trampolines read this pointer from interrupt context so
    //    it must be set before the task (and therefore interrupts) start.
    // ------------------------------------------------------------------
    s_instance = this;

    // ------------------------------------------------------------------
    // 7. Create the radio state-machine task using static allocation.
    //    taskEntryStatic bridges the C-style void* arg to our C++ instance.
    // ------------------------------------------------------------------
    _taskHandle = xTaskCreateStatic(
        taskEntryStatic,
        "lora_radio",
        CONFIG_LORA_TASK_STACK_SIZE,
        this,                           // passed as arg → cast back to LoraRadio*
        _cfg.taskPriority,
        s_taskStack,
        &s_taskTcb
    );

    if (!_taskHandle) {
        ESP_LOGE(TAG, "xTaskCreateStatic failed");
        return ESP_FAIL;
    }

    _initialized = true;
    ESP_LOGI(TAG, "Radio task started (priority %lu, stack %lu B)",
             _cfg.taskPriority, _cfg.taskStackSize);
    return ESP_OK;
}

void LoraRadio::deinit()
{
    if (!_initialized) return;

    // Signal the task to stop and wait for it
    if (_taskHandle) {
        xTaskNotify(_taskHandle, NOTIFY_STOP, eSetBits);
        // Give the task up to 2 s to exit cleanly
        vTaskDelay(pdMS_TO_TICKS(2000));
        _taskHandle = nullptr;
    }

    s_instance = nullptr;

    // Detach ISRs through RadioLib
    if (_radio) {
        toRadio(_radio)->clearDio0Action();
        if (_cfg.pinDio1 != RADIOLIB_NC) {
            toRadio(_radio)->clearDio1Action();
        }
        toRadio(_radio)->sleep();
        delete toRadio(_radio);
        _radio = nullptr;
    }

    if (_module) {
        delete toModule(_module);
        _module = nullptr;
    }

    // HAL destructor calls term() → spi_bus_remove_device + spi_bus_free (RAII)
    if (_hal) {
        delete _hal;
        _hal = nullptr;
    }

    _initialized = false;
    ESP_LOGI(TAG, "Radio deinitialized");
}

// ===========================================================================
// Public TX API  (thread-safe — callable from any task)
// ===========================================================================

esp_err_t LoraRadio::send(uint16_t       dstId,
                           uint8_t        msgType,
                           const uint8_t* data,
                           size_t         len,
                           bool           requestAck)
{
    if (len > LORA_MAX_PAYLOAD) {
        ESP_LOGW(TAG, "send(): payload too large (%zu > %zu)", len, LORA_MAX_PAYLOAD);
        return ESP_ERR_INVALID_SIZE;
    }

    TxItem item{};
    // Serialise the packet header into item.buf
    PacketHeader* hdr = reinterpret_cast<PacketHeader*>(item.buf);
    hdr->srcId      = _cfg.nodeId;
    hdr->dstId      = dstId;
    hdr->seqNum     = _seqNum++;
    hdr->msgType    = msgType;
    hdr->flags      = requestAck ? FLAG_ACK_REQUEST : 0;
    hdr->payloadLen = static_cast<uint8_t>(len);

    if (len > 0 && data) {
        std::memcpy(item.buf + PACKET_HEADER_SIZE, data, len);
    }
    item.len      = static_cast<uint16_t>(PACKET_HEADER_SIZE + len);
    item.retries  = 0;
    item.needsAck = requestAck;

    // Push to normal queue (non-blocking)
    if (xQueueSend(_normalQueue, &item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "send(): normal TX queue full");
        return ESP_ERR_NO_MEM;
    }

    // Wake sleeping radio if needed
    if (_state == State::SLEEPING && _taskHandle) {
        xTaskNotify(_taskHandle, NOTIFY_TX_QUEUED, eSetBits);
    }
    return ESP_OK;
}

void LoraRadio::setRxCallback(RxCallback cb)
{
    // Simple pointer write — atomic on 32-bit Xtensa (no mutex needed for
    // a single-writer single-reader function pointer)
    _rxCb = cb;
}

size_t LoraRadio::getNeighbors(NeighborEntry* out, size_t maxCount) const
{
    size_t written = 0;
    for (size_t i = 0; i < CONFIG_LORA_MAX_NEIGHBORS && written < maxCount; ++i) {
        if (s_neighbors[i].valid) {
            out[written++] = s_neighbors[i];
        }
    }
    return written;
}

// ===========================================================================
// ISR trampolines  (IRAM_ATTR — keeps them callable during flash operations)
// ===========================================================================
// These are the ONLY functions that run in interrupt context.
// Contract: zero SPI transactions, zero blocking calls, zero logic.
// Responsibility: post a notification bit and yield to the woken task.
// ===========================================================================

void IRAM_ATTR LoraRadio::dio0Isr()
{
    // DIO0 fires on: TX done, RX done, CAD no-activity (channel clear).
    // The state-machine task reads _state to know which event this is.
    BaseType_t higherPrioTaskWoken = pdFALSE;
    xTaskNotifyFromISR(s_instance->_taskHandle,
                       NOTIFY_DIO0,
                       eSetBits,
                       &higherPrioTaskWoken);
    // Yield to the radio task if it has higher priority than the current one
    portYIELD_FROM_ISR(higherPrioTaskWoken);
}

void IRAM_ATTR LoraRadio::dio1Isr()
{
    // DIO1 fires on: CAD preamble detected (channel busy).
    BaseType_t higherPrioTaskWoken = pdFALSE;
    xTaskNotifyFromISR(s_instance->_taskHandle,
                       NOTIFY_DIO1,
                       eSetBits,
                       &higherPrioTaskWoken);
    portYIELD_FROM_ISR(higherPrioTaskWoken);
}

// ===========================================================================
// Task entry point — bridges C-style void* arg to C++ this pointer
// ===========================================================================

void LoraRadio::taskEntryStatic(void* arg)
{
    // Cast the opaque arg back to the LoraRadio instance and run.
    // This pattern is the standard way to use xTaskCreateStatic with a
    // C++ class method as the task body.
    static_cast<LoraRadio*>(arg)->taskLoop();
    // taskLoop() returns only when NOTIFY_STOP is received
    vTaskDelete(nullptr);
}

// ===========================================================================
// Helper: wait for task notifications with a timeout
// Returns the bitmask of notification bits that were set.
// Clears all bits on return so the next wait starts fresh.
// ===========================================================================

uint32_t LoraRadio::waitNotify(uint32_t timeoutMs)
{
    uint32_t bits = 0;
    xTaskNotifyWait(
        0,                          // don't clear bits on entry
        NOTIFY_ALL_BITS,            // clear all bits on exit
        &bits,
        pdMS_TO_TICKS(timeoutMs)
    );
    return bits;
}

// ===========================================================================
// Helper: dequeue the next TX item (emergency queue first)
// ===========================================================================

bool LoraRadio::dequeueNextTx(TxItem& out)
{
    return xQueueReceive(_normalQueue, &out, 0) == pdTRUE;
}

// ===========================================================================
// Helper: build an ACK frame into frameBuf / frameLen
// ===========================================================================

void LoraRadio::buildAck(uint8_t* frameBuf, uint16_t& frameLen,
                          const PacketHeader& rxHdr)
{
    PacketHeader* ack = reinterpret_cast<PacketHeader*>(frameBuf);
    ack->srcId      = _cfg.nodeId;
    ack->dstId      = rxHdr.srcId;
    ack->seqNum     = rxHdr.seqNum;      // echo back seqNum for matching
    ack->msgType    = MSGTYPE_ACK;
    ack->flags      = 0;
    ack->payloadLen = 0;
    frameLen        = static_cast<uint16_t>(PACKET_HEADER_SIZE);
}

// ===========================================================================
// Helper: update the neighbour table on reception
// ===========================================================================

void LoraRadio::updateNeighbor(uint16_t nodeId, float rssi, float snr)
{
    uint32_t nowMs = (uint32_t)(esp_timer_get_time() / 1000ULL);
    int freeSlot = -1;

    for (size_t i = 0; i < CONFIG_LORA_MAX_NEIGHBORS; ++i) {
        if (s_neighbors[i].valid && s_neighbors[i].nodeId == nodeId) {
            // Update existing entry
            s_neighbors[i].rssi       = rssi;
            s_neighbors[i].snr        = snr;
            s_neighbors[i].lastSeenMs = nowMs;
            return;
        }
        if (!s_neighbors[i].valid && freeSlot < 0) {
            freeSlot = (int)i;
        }
    }

    // New neighbour — find a slot (LRU eviction if table is full)
    if (freeSlot < 0) {
        // Evict the oldest seen entry
        uint32_t oldest = UINT32_MAX;
        for (size_t i = 0; i < CONFIG_LORA_MAX_NEIGHBORS; ++i) {
            if (s_neighbors[i].lastSeenMs < oldest) {
                oldest   = s_neighbors[i].lastSeenMs;
                freeSlot = (int)i;
            }
        }
    }

    s_neighbors[freeSlot] = { nodeId, rssi, snr, nowMs, true };
    ESP_LOGI(TAG, "New neighbour 0x%04X  RSSI=%.1f SNR=%.1f", nodeId, (double)rssi, (double)snr);
}

// ===========================================================================
// Helper: parse a received frame and fire callbacks / send ACK
// ===========================================================================

void LoraRadio::dispatchRx(const uint8_t* frame, size_t len,
                            float rssi, float snr)
{
    if (len < PACKET_HEADER_SIZE) {
        ESP_LOGD(TAG, "RX: frame too short (%zu bytes)", len);
        return;
    }

    const PacketHeader* hdr =
        reinterpret_cast<const PacketHeader*>(frame);

    ESP_LOGD(TAG, "RX: src=0x%04X dst=0x%04X seq=%u type=%u flags=0x%02X len=%u "
             "RSSI=%.1f SNR=%.1f",
             hdr->srcId, hdr->dstId, hdr->seqNum, hdr->msgType,
             hdr->flags, hdr->payloadLen, (double)rssi, (double)snr);

    // Update neighbour table (all received packets contribute)
    updateNeighbor(hdr->srcId, rssi, snr);

    // Drop packets not addressed to us (unless broadcast)
    if (hdr->dstId != 0xFFFF && hdr->dstId != _cfg.nodeId) {
        return;
    }

    // ACK packets are handled in WAIT_ACK state — do not re-process here
    if (hdr->msgType == MSGTYPE_ACK) {
        return;
    }

    // If sender requested an ACK, build one and queue it for transmission.
    // We push to the normal queue here (task context, safe); the CAD→TX
    // path will transmit it on the next IDLE cycle.
    if (hdr->flags & FLAG_ACK_REQUEST) {
        TxItem ackItem{};
        buildAck(ackItem.buf, ackItem.len, *hdr);
        ackItem.needsAck = false;
        if (xQueueSend(_normalQueue, &ackItem, 0) != pdTRUE) {
            ESP_LOGW(TAG, "dispatchRx: TX queue full, ACK dropped");
        }
    }

    // Fire the application-layer receive callback
    if (_rxCb) {
        const uint8_t* payload = frame + PACKET_HEADER_SIZE;
        size_t payloadLen = (hdr->payloadLen < LORA_MAX_PAYLOAD)
                            ? hdr->payloadLen : LORA_MAX_PAYLOAD;
        _rxCb(*hdr, payload, rssi, snr);
        (void)payloadLen;
    }
}

// ===========================================================================
// Main state-machine task loop
// ===========================================================================

void LoraRadio::taskLoop()
{
    SX1278* radio = toRadio(_radio);
    ESP_LOGI(TAG, "State machine started");

    _state = State::IDLE;

    while (true) {

        // ----------------------------------------------------------------
        // IDLE — decide what to do next
        // ----------------------------------------------------------------
        if (_state == State::IDLE) {
            // Check for a STOP signal first
            uint32_t bits = 0;
            xTaskNotifyWait(0, NOTIFY_STOP, &bits, 0);
            if (bits & NOTIFY_STOP) {
                ESP_LOGI(TAG, "STOP received — exiting task loop");
                return;
            }

            if (dequeueNextTx(_currentTx)) {
                // Start Channel Activity Detection before transmitting.
                // DIO0 → no preamble (channel clear)
                // DIO1 → preamble detected (channel busy)
                _hasPendingTx = true;
                _cadRetries   = 0;
                ESP_LOGD(TAG, "IDLE→CAD: queued %u bytes", _currentTx.len);
                int16_t cadErr = radio->startChannelScan();
                if (cadErr != RADIOLIB_ERR_NONE) {
                    ESP_LOGW(TAG, "startChannelScan failed (%d), skipping CAD", cadErr);
                    // Fall through to TX directly if CAD unavailable
                    _state = State::TX_WAIT;
                    radio->startTransmit(_currentTx.buf, _currentTx.len);
                } else {
                    _state = State::CAD;
                }
            } else {
                // Nothing to send — start background receive.
                // The modem will fire DIO0 when a full packet is received.
                radio->startReceive();
                _state = State::RX;
            }
        }

        // ----------------------------------------------------------------
        // CAD — wait for channel activity detection result
        // ----------------------------------------------------------------
        else if (_state == State::CAD) {
            // Use a generous timeout; the SX1278 CAD takes < 1 symbol time
            uint32_t bits = waitNotify(500);

            if (bits & NOTIFY_STOP) { return; }

            if (bits & NOTIFY_DIO1) {
                // DIO1: preamble detected → channel is busy
                ESP_LOGD(TAG, "CAD: channel busy (retry %u/%u)",
                         _cadRetries + 1, (unsigned)_cfg.cadRetries);

                if (++_cadRetries < _cfg.cadRetries) {
                    // Exponential backoff: wait 50–200 ms then retry CAD
                    uint32_t backoffMs = 50 + (uint32_t)(rand() % 150);
                    vTaskDelay(pdMS_TO_TICKS(backoffMs));
                    radio->startChannelScan();
                    // Stay in CAD state
                } else {
                    // Max retries reached — drop the packet to prevent starvation
                    ESP_LOGW(TAG, "CAD: max retries reached, dropping packet");
                    _hasPendingTx = false;
                    _state = State::IDLE;
                }
            } else if (bits & NOTIFY_DIO0) {
                // DIO0: CadDone — read result
                int16_t result = radio->getChannelScanResult();
                if (result == RADIOLIB_LORA_DETECTED) {
                    // Channel occupied even after DIO0 — treat as busy
                    ESP_LOGD(TAG, "CAD: LORA_DETECTED on DIO0, retrying");
                    if (++_cadRetries < _cfg.cadRetries) {
                        uint32_t backoffMs = 50 + (uint32_t)(rand() % 150);
                        vTaskDelay(pdMS_TO_TICKS(backoffMs));
                        radio->startChannelScan();
                    } else {
                        ESP_LOGW(TAG, "CAD: max retries, dropping");
                        _hasPendingTx = false;
                        _state = State::IDLE;
                    }
                } else {
                    // RADIOLIB_CHANNEL_FREE — transmit now
                    ESP_LOGD(TAG, "CAD: channel clear → TX");
                    radio->startTransmit(_currentTx.buf, _currentTx.len);
                    _state = State::TX_WAIT;
                }
            } else {
                // Timeout — assume clear and transmit (edge-case: DIO pin not wired)
                ESP_LOGD(TAG, "CAD: timeout, assuming clear → TX");
                radio->startTransmit(_currentTx.buf, _currentTx.len);
                _state = State::TX_WAIT;
            }
        }

        // ----------------------------------------------------------------
        // TX_WAIT — radio is transmitting; wait for DIO0 (TxDone)
        // ----------------------------------------------------------------
        else if (_state == State::TX_WAIT) {
            uint32_t bits = waitNotify(5000);   // 5 s hard timeout

            if (bits & NOTIFY_STOP) { return; }

            if (bits & NOTIFY_DIO0) {
                // Transmission complete — housekeeping
                radio->finishTransmit();
                ESP_LOGD(TAG, "TX done: %u bytes", _currentTx.len);

                if (_currentTx.needsAck) {
                    // Park the item in _pendingAck and listen for reply
                    _pendingAck  = _currentTx;
                    _waitingAck  = true;
                    radio->startReceive();
                    _state = State::WAIT_ACK;
                } else {
                    _hasPendingTx = false;
                    _state = State::IDLE;
                }
            } else {
                // Timeout — radio may have hung; reset to IDLE
                ESP_LOGW(TAG, "TX_WAIT timeout — resetting");
                radio->standby();
                _hasPendingTx = false;
                _state = State::IDLE;
            }
        }

        // ----------------------------------------------------------------
        // WAIT_ACK — listening for ACK after a needsAck TX
        // ----------------------------------------------------------------
        else if (_state == State::WAIT_ACK) {
            uint32_t bits = waitNotify(_cfg.ackTimeoutMs);

            if (bits & NOTIFY_STOP) { return; }

            if (bits & NOTIFY_DIO0) {
                // A packet arrived — check if it is our ACK
                size_t pktLen = radio->getPacketLength();
                uint8_t buf[MAX_FRAME_SIZE] = {};
                if (pktLen > 0 && pktLen <= MAX_FRAME_SIZE) {
                    radio->readData(buf, pktLen);
                    float rssi = radio->getRSSI();
                    float snr  = radio->getSNR();

                    if (pktLen >= PACKET_HEADER_SIZE) {
                        const PacketHeader* rxHdr =
                            reinterpret_cast<const PacketHeader*>(buf);

                        if (rxHdr->msgType == MSGTYPE_ACK
                            && rxHdr->seqNum == _pendingAck.buf[
                                offsetof(PacketHeader, seqNum)]
                            && rxHdr->dstId  == _cfg.nodeId)
                        {
                            // Correct ACK received
                            ESP_LOGD(TAG, "ACK received for seq=%u", rxHdr->seqNum);
                            updateNeighbor(rxHdr->srcId, rssi, snr);
                            _waitingAck   = false;
                            _hasPendingTx = false;
                            _state = State::IDLE;
                        } else {
                            // Some other packet — dispatch it, keep waiting
                            dispatchRx(buf, pktLen, rssi, snr);
                            radio->startReceive();
                            // State stays WAIT_ACK; effective timeout restarts
                        }
                    }
                }
            } else {
                // Timeout — retransmit or give up
                if (_pendingAck.retries < _cfg.maxRetries) {
                    ++_pendingAck.retries;
                    PacketHeader* hdr =
                        reinterpret_cast<PacketHeader*>(_pendingAck.buf);
                    hdr->flags |= FLAG_IS_RETRANSMIT;
                    ESP_LOGD(TAG, "ACK timeout: retransmit %u/%lu",
                             _pendingAck.retries,
                             _cfg.maxRetries);
                    _currentTx    = _pendingAck;
                    _cadRetries   = 0;
                    radio->startChannelScan();
                    _state = State::CAD;
                } else {
                    ESP_LOGW(TAG, "ACK: max retries (%lu) exhausted, dropping",
                             _cfg.maxRetries);
                    _waitingAck   = false;
                    _hasPendingTx = false;
                    _state = State::IDLE;
                }
            }
        }

        // ----------------------------------------------------------------
        // RX — radio is in continuous receive; wait for DIO0 (RxDone)
        //       or idle timeout → sleep
        // ----------------------------------------------------------------
        else if (_state == State::RX) {
            uint32_t bits = waitNotify(_cfg.sleepIdleMs);

            if (bits & NOTIFY_STOP)      { return; }
            if (bits & NOTIFY_TX_QUEUED) { radio->standby(); _state = State::IDLE; continue; }

            if (bits & NOTIFY_DIO0) {
                // Full packet received
                size_t pktLen = radio->getPacketLength();
                uint8_t buf[MAX_FRAME_SIZE] = {};
                if (pktLen > 0 && pktLen <= MAX_FRAME_SIZE) {
                    int16_t rxErr = radio->readData(buf, pktLen);
                    if (rxErr == RADIOLIB_ERR_NONE) {
                        float rssi = radio->getRSSI();
                        float snr  = radio->getSNR();
                        dispatchRx(buf, pktLen, rssi, snr);
                    } else {
                        ESP_LOGD(TAG, "readData error %d", rxErr);
                    }
                }
                // Re-check TX queue; if nothing, go back to receive in IDLE
                _state = State::IDLE;
            } else {
                // Idle timeout — enter modem sleep to save power
                ESP_LOGD(TAG, "Idle timeout → SLEEPING");
                radio->sleep();
                _state = State::SLEEPING;
            }
        }

        // ----------------------------------------------------------------
        // SLEEPING — modem in low-power sleep; wake on TX_QUEUED or STOP
        // ----------------------------------------------------------------
        else if (_state == State::SLEEPING) {
            ESP_LOGI(TAG, "Modem sleeping (idle >%lu ms)", _cfg.sleepIdleMs);

            // Wait indefinitely — only TX_QUEUED or STOP can wake us
            uint32_t bits = waitNotify(portMAX_DELAY);

            if (bits & NOTIFY_STOP) { return; }

            if (bits & (NOTIFY_TX_QUEUED | NOTIFY_DIO0)) {
                // Wake the modem and go back to IDLE to process the queue
                radio->standby();
                ESP_LOGI(TAG, "Modem wake-up");
                _state = State::IDLE;
            }
        }

    } // while(true)
}

// ===========================================================================
// Private member _rxCb must be declared — add storage here
// (The field was declared in the header in the private section; the linker
//  needs a definition for function-pointer types in some toolchain versions)
// ===========================================================================
// Note: This is just a reminder comment — the field is inline in the class.
// No out-of-line definition is needed for a non-static non-const member.
