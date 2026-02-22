/**
 * @file lora_radio.cpp
 * @brief LoRa V2V radio state machine and FreeRTOS task implementation.
 *
 * @details
 * Implementation notes:
 *  - The concrete radio hardware is accessed exclusively through IRadioBackend.
 *    Production builds use Sx1278Backend (wrapping RadioLib SX1278).
 *    Test builds inject MockRadioBackend via initForTest().
 *  - RadioLib objects (EspHal, Module, SX1278, Sx1278Backend) are heap-
 *    allocated exactly once inside init() and freed inside deinit().
 *    In test mode (initForTest), none of these are created.
 *  - All FreeRTOS objects (task, queues) use static storage declared below.
 *    xTaskCreateStatic / xQueueCreateStatic never call pvPortMalloc.
 *  - DIO0 and DIO1 ISRs contain only xTaskNotifyFromISR + portYIELD_FROM_ISR.
 *    Zero SPI transactions, zero blocking calls, zero logic in ISR context.
 */

// sx1278_backend.hpp brings in <RadioLib.h> for the production init() path.
// The state-machine code (taskLoop, dispatchRx, etc.) never touches RadioLib
// directly; all calls go through the IRadioBackend interface.
#include "sx1278_backend.hpp"

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

StackType_t  LoraRadio::s_taskStack[CONFIG_LORA_TASK_STACK_SIZE];
StaticTask_t LoraRadio::s_taskTcb;

uint8_t       LoraRadio::s_normalQueueStorage[
    CONFIG_LORA_TX_QUEUE_DEPTH * sizeof(LoraRadio::TxItem)];
StaticQueue_t LoraRadio::s_normalQueueState;

NeighborEntry LoraRadio::s_neighbors[CONFIG_LORA_MAX_NEIGHBORS];

LoraRadio*          LoraRadio::s_instance        = nullptr;
volatile bool       LoraRadio::s_taskRunning     = false;
TaskHandle_t        LoraRadio::s_runningTaskHandle = nullptr;

// ===========================================================================
// Cast helper (keeps RadioLib headers out of lora_radio.hpp)
// ===========================================================================

static inline Module* toModule(void* p) { return reinterpret_cast<Module*>(p); }
static inline SX1278* toSX1278(void* p) { return reinterpret_cast<SX1278*>(p); }

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
// Lifecycle — production path (real SX1278 hardware)
// ===========================================================================

esp_err_t LoraRadio::init()
{
    if (_initialized) {
        ESP_LOGW(TAG, "init() called twice — ignoring");
        return ESP_OK;
    }

    // ------------------------------------------------------------------
    // 1. Construct and initialise the ESP-IDF SPI HAL.
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
    //    cs  = NSS pin, irq = DIO0, rst = hardware reset,
    //    gpio = RADIOLIB_NC (DIO1 attached manually below via setDio1Action)
    // ------------------------------------------------------------------
    _module = new Module(
        _hal,
        _cfg.pinNss,
        _cfg.pinDio0,
        _cfg.pinRst,
        RADIOLIB_NC
    );

    // ------------------------------------------------------------------
    // 3. Construct SX1278 and wrap it in Sx1278Backend.
    // ------------------------------------------------------------------
    SX1278* radio = new SX1278(toModule(_module));
    _sx1278Raw    = radio;          // keep raw pointer for deinit()
    _ownsBackend  = true;

    ESP_LOGI(TAG, "Initialising SX1278 @ %.3f MHz  SF%u  BW%.0fkHz  CR4/%u  +%ddBm",
             (double)_cfg.frequencyMHz,
             _cfg.spreadingFactor,
             (double)_cfg.bandwidthKHz,
             _cfg.codingRate,
             _cfg.txPowerDbm);

    return _initCommon(new Sx1278Backend(radio));
}

// ===========================================================================
// Lifecycle — shared init logic (called by both init() and initForTest())
// ===========================================================================

esp_err_t LoraRadio::_initCommon(IRadioBackend* backend)
{
    _backend = backend;

    // ------------------------------------------------------------------
    // Configure RF parameters and bring up the modem.
    // ------------------------------------------------------------------
    int16_t state = _backend->begin(
        _cfg.frequencyMHz,
        _cfg.bandwidthKHz,
        _cfg.spreadingFactor,
        _cfg.codingRate,
        _cfg.syncWord,
        _cfg.txPowerDbm
    );

    if (state != RADIO_ERR_NONE) {
        ESP_LOGE(TAG, "radio begin() failed, code %d", state);

        // Release ownership so deinit() does not double-free
        if (_ownsBackend) {
            delete _backend;
        }
        _backend     = nullptr;
        _ownsBackend = false;

        if (_sx1278Raw) { delete toSX1278(_sx1278Raw); _sx1278Raw = nullptr; }
        if (_module)    { delete toModule(_module);    _module    = nullptr; }
        if (_hal)       { delete _hal;                 _hal       = nullptr; }

        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Radio init OK");

    // ------------------------------------------------------------------
    // Attach ISR trampolines via the backend.
    // DIO0: TX done / RX done / CAD no-activity
    // DIO1: CAD preamble detected (channel busy)
    // ------------------------------------------------------------------
    _backend->setDio0Action(LoraRadio::dio0Isr, RADIO_RISING);

    if (_cfg.pinDio1 != RADIO_PIN_NC) {
        _backend->setDio1Action(LoraRadio::dio1Isr, RADIO_RISING);
    } else {
        ESP_LOGW(TAG, "DIO1 not connected — CAD busy detection disabled");
    }

    // ------------------------------------------------------------------
    // Create statically-allocated FreeRTOS TX queue.
    // ------------------------------------------------------------------
    _normalQueue = xQueueCreateStatic(
        CONFIG_LORA_TX_QUEUE_DEPTH,
        sizeof(TxItem),
        s_normalQueueStorage,
        &s_normalQueueState
    );

    // ------------------------------------------------------------------
    // Set singleton pointer before creating the task so ISRs can reach
    // the task handle from the moment interrupts are enabled.
    // ------------------------------------------------------------------
    s_instance = this;

    // ------------------------------------------------------------------
    // Wait for any previously-created task to finish using the shared
    // static stack / TCB buffers before re-using them.  This is normally
    // instant (s_taskRunning starts false) but protects the rare case
    // where Unity longjmps past a test-fixture destructor, leaving the
    // previous task alive when the next test re-initialises the radio.
    //
    // If a task is still alive (e.g., stuck in TX_WAIT or SLEEPING after
    // a longjmp), signal it to exit before waiting, otherwise it may
    // block portMAX_DELAY with nothing left to wake it.
    // ------------------------------------------------------------------
    if (s_taskRunning && s_runningTaskHandle) {
        xTaskNotify(s_runningTaskHandle, NOTIFY_STOP, eSetBits);
    }
    while (s_taskRunning) { vTaskDelay(pdMS_TO_TICKS(10)); }

    // ------------------------------------------------------------------
    // Create the radio state-machine task using static allocation.
    // ------------------------------------------------------------------
    _taskHandle = xTaskCreateStatic(
        taskEntryStatic,
        "lora_radio",
        CONFIG_LORA_TASK_STACK_SIZE,
        this,
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

// ===========================================================================
// Lifecycle — shutdown
// ===========================================================================

void LoraRadio::deinit()
{
    if (!_initialized) return;

    // Signal the task to stop and wait up to 2 s
    if (_taskHandle) {
        xTaskNotify(_taskHandle, NOTIFY_STOP, eSetBits);
        vTaskDelay(pdMS_TO_TICKS(2000));
        _taskHandle = nullptr;
    }

    s_instance = nullptr;

    if (_backend) {
        _backend->clearDio0Action();
        if (_cfg.pinDio1 != RADIO_PIN_NC) {
            _backend->clearDio1Action();
        }
        _backend->sleep();

        if (_ownsBackend) {
            delete _backend;
        }
        _backend     = nullptr;
        _ownsBackend = false;
    }

    // Free the raw hardware objects (nullptr in test mode)
    if (_sx1278Raw) { delete toSX1278(_sx1278Raw); _sx1278Raw = nullptr; }
    if (_module)    { delete toModule(_module);    _module    = nullptr; }
    if (_hal)       { delete _hal;                 _hal       = nullptr; }

    _initialized = false;
    ESP_LOGI(TAG, "Radio deinitialized");
}

// ===========================================================================
// Test seam  (compiled only when CONFIG_LORA_ENABLE_TEST_SEAM=y)
// ===========================================================================

#if CONFIG_LORA_ENABLE_TEST_SEAM

esp_err_t LoraRadio::initForTest(IRadioBackend* backend)
{
    if (_initialized) {
        ESP_LOGW(TAG, "initForTest() called twice — ignoring");
        return ESP_OK;
    }
    // In test mode: no EspHal, Module, or SX1278 are created.
    // The backend is externally owned; deinit() must not delete it.
    _ownsBackend = false;
    return _initCommon(backend);
}

void LoraRadio::injectNeighborUpdate(uint16_t nodeId, float rssi, float snr)
{
    updateNeighbor(nodeId, rssi, snr);
}

#endif // CONFIG_LORA_ENABLE_TEST_SEAM

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

    if (xQueueSend(_normalQueue, &item, 0) != pdTRUE) {
        ESP_LOGW(TAG, "send(): normal TX queue full");
        return ESP_ERR_NO_MEM;
    }

    // Wake the radio task if it is idle-sleeping or already in receive mode.
    // In RX state the task handles NOTIFY_TX_QUEUED by calling standby() then
    // looping back to IDLE where it will dequeue and start CAD.
    if ((_state == State::SLEEPING || _state == State::RX) && _taskHandle) {
        xTaskNotify(_taskHandle, NOTIFY_TX_QUEUED, eSetBits);
    }
    return ESP_OK;
}

void LoraRadio::setRxCallback(RxCallback cb)
{
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

void IRAM_ATTR LoraRadio::dio0Isr()
{
    if (!s_instance || !s_instance->_taskHandle) return;
    if (xPortInIsrContext()) {
        BaseType_t x = pdFALSE;
        xTaskNotifyFromISR(s_instance->_taskHandle, NOTIFY_DIO0, eSetBits, &x);
        portYIELD_FROM_ISR(x);
    } else {
        // Called from task context (e.g., mock in unit tests) — use task-safe API.
        xTaskNotify(s_instance->_taskHandle, NOTIFY_DIO0, eSetBits);
    }
}

void IRAM_ATTR LoraRadio::dio1Isr()
{
    if (!s_instance || !s_instance->_taskHandle) return;
    if (xPortInIsrContext()) {
        BaseType_t x = pdFALSE;
        xTaskNotifyFromISR(s_instance->_taskHandle, NOTIFY_DIO1, eSetBits, &x);
        portYIELD_FROM_ISR(x);
    } else {
        // Called from task context (e.g., mock in unit tests) — use task-safe API.
        xTaskNotify(s_instance->_taskHandle, NOTIFY_DIO1, eSetBits);
    }
}

// ===========================================================================
// Task entry point
// ===========================================================================

void LoraRadio::taskEntryStatic(void* arg)
{
    LoraRadio* self = static_cast<LoraRadio*>(arg);
    s_taskRunning      = true;
    s_runningTaskHandle = self->_taskHandle;
    self->taskLoop();
    s_runningTaskHandle = nullptr;
    s_taskRunning       = false;
    vTaskDelete(nullptr);
}

// ===========================================================================
// Helper: wait for task notifications with a timeout
// ===========================================================================

uint32_t LoraRadio::waitNotify(uint32_t timeoutMs)
{
    uint32_t bits = 0;
    xTaskNotifyWait(
        0,
        NOTIFY_ALL_BITS,
        &bits,
        pdMS_TO_TICKS(timeoutMs)
    );
    return bits;
}

// ===========================================================================
// Helper: dequeue the next TX item
// ===========================================================================

bool LoraRadio::dequeueNextTx(TxItem& out)
{
    return xQueueReceive(_normalQueue, &out, 0) == pdTRUE;
}

// ===========================================================================
// Helper: build an ACK frame
// ===========================================================================

void LoraRadio::buildAck(uint8_t* frameBuf, uint16_t& frameLen,
                          const PacketHeader& rxHdr)
{
    PacketHeader* ack = reinterpret_cast<PacketHeader*>(frameBuf);
    ack->srcId      = _cfg.nodeId;
    ack->dstId      = rxHdr.srcId;
    ack->seqNum     = rxHdr.seqNum;
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
            s_neighbors[i].rssi       = rssi;
            s_neighbors[i].snr        = snr;
            s_neighbors[i].lastSeenMs = nowMs;
            return;
        }
        if (!s_neighbors[i].valid && freeSlot < 0) {
            freeSlot = (int)i;
        }
    }

    if (freeSlot < 0) {
        uint32_t oldest = UINT32_MAX;
        for (size_t i = 0; i < CONFIG_LORA_MAX_NEIGHBORS; ++i) {
            if (s_neighbors[i].lastSeenMs < oldest) {
                oldest   = s_neighbors[i].lastSeenMs;
                freeSlot = (int)i;
            }
        }
    }

    s_neighbors[freeSlot] = { nodeId, rssi, snr, nowMs, true };
    ESP_LOGI(TAG, "New neighbour 0x%04X  RSSI=%.1f SNR=%.1f",
             nodeId, (double)rssi, (double)snr);
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

    const PacketHeader* hdr = reinterpret_cast<const PacketHeader*>(frame);

    ESP_LOGD(TAG, "RX: src=0x%04X dst=0x%04X seq=%u type=%u flags=0x%02X len=%u "
             "RSSI=%.1f SNR=%.1f",
             hdr->srcId, hdr->dstId, hdr->seqNum, hdr->msgType,
             hdr->flags, hdr->payloadLen, (double)rssi, (double)snr);

    updateNeighbor(hdr->srcId, rssi, snr);

    if (hdr->dstId != 0xFFFF && hdr->dstId != _cfg.nodeId) {
        return;
    }

    if (hdr->msgType == MSGTYPE_ACK) {
        return;
    }

    if (hdr->flags & FLAG_ACK_REQUEST) {
        TxItem ackItem{};
        buildAck(ackItem.buf, ackItem.len, *hdr);
        ackItem.needsAck = false;
        if (xQueueSend(_normalQueue, &ackItem, 0) != pdTRUE) {
            ESP_LOGW(TAG, "dispatchRx: TX queue full, ACK dropped");
        }
    }

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
    // All radio calls go through the backend interface.
    // In production: _backend is a Sx1278Backend wrapping a real SX1278.
    // In test mode:  _backend is a MockRadioBackend injected by initForTest().
    IRadioBackend* radio = _backend;

    ESP_LOGI(TAG, "State machine started");
    _state = State::IDLE;

    while (true) {

        // ----------------------------------------------------------------
        // IDLE — decide what to do next
        // ----------------------------------------------------------------
        if (_state == State::IDLE) {
            uint32_t bits = 0;
            xTaskNotifyWait(0, NOTIFY_STOP, &bits, 0);
            if (bits & NOTIFY_STOP) {
                ESP_LOGI(TAG, "STOP received — exiting task loop");
                return;
            }

            if (dequeueNextTx(_currentTx)) {
                _hasPendingTx = true;
                _cadRetries   = 0;
                ESP_LOGD(TAG, "IDLE→CAD: queued %u bytes", _currentTx.len);
                int16_t cadErr = radio->startChannelScan();
                if (cadErr != RADIO_ERR_NONE) {
                    ESP_LOGW(TAG, "startChannelScan failed (%d), skipping CAD", cadErr);
                    _state = State::TX_WAIT;
                    radio->startTransmit(_currentTx.buf, _currentTx.len);
                } else {
                    _state = State::CAD;
                }
            } else {
                radio->startReceive();
                _state = State::RX;
            }
        }

        // ----------------------------------------------------------------
        // CAD — wait for channel activity detection result
        // ----------------------------------------------------------------
        else if (_state == State::CAD) {
            uint32_t bits = waitNotify(500);

            if (bits & NOTIFY_STOP) { return; }

            if (bits & NOTIFY_DIO1) {
                ESP_LOGD(TAG, "CAD: channel busy (retry %u/%u)",
                         _cadRetries + 1, (unsigned)_cfg.cadRetries);

                if (++_cadRetries < _cfg.cadRetries) {
                    uint32_t backoffMs = 50 + (uint32_t)(rand() % 150);
                    vTaskDelay(pdMS_TO_TICKS(backoffMs));
                    radio->startChannelScan();
                } else {
                    ESP_LOGW(TAG, "CAD: max retries reached, dropping packet");
                    _hasPendingTx = false;
                    _state = State::IDLE;
                }
            } else if (bits & NOTIFY_DIO0) {
                int16_t result = radio->getChannelScanResult();
                if (result == RADIO_LORA_DETECTED) {
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
                    ESP_LOGD(TAG, "CAD: channel clear → TX");
                    radio->startTransmit(_currentTx.buf, _currentTx.len);
                    _state = State::TX_WAIT;
                }
            } else {
                // Timeout — assume clear
                ESP_LOGD(TAG, "CAD: timeout, assuming clear → TX");
                radio->startTransmit(_currentTx.buf, _currentTx.len);
                _state = State::TX_WAIT;
            }
        }

        // ----------------------------------------------------------------
        // TX_WAIT — radio is transmitting; wait for DIO0 (TxDone)
        // ----------------------------------------------------------------
        else if (_state == State::TX_WAIT) {
            uint32_t bits = waitNotify(5000);

            if (bits & NOTIFY_STOP) { return; }

            if (bits & NOTIFY_DIO0) {
                radio->finishTransmit();
                ESP_LOGD(TAG, "TX done: %u bytes", _currentTx.len);

                if (_currentTx.needsAck) {
                    _pendingAck  = _currentTx;
                    _waitingAck  = true;
                    radio->startReceive();
                    _state = State::WAIT_ACK;
                } else {
                    _hasPendingTx = false;
                    _state = State::IDLE;
                }
            } else {
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
                            ESP_LOGD(TAG, "ACK received for seq=%u", rxHdr->seqNum);
                            updateNeighbor(rxHdr->srcId, rssi, snr);
                            _waitingAck   = false;
                            _hasPendingTx = false;
                            _state = State::IDLE;
                        } else {
                            dispatchRx(buf, pktLen, rssi, snr);
                            radio->startReceive();
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
                             _pendingAck.retries, _cfg.maxRetries);
                    _currentTx  = _pendingAck;
                    _cadRetries = 0;
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
        // RX — radio in continuous receive; wait for DIO0 or idle timeout
        // ----------------------------------------------------------------
        else if (_state == State::RX) {
            uint32_t bits = waitNotify(_cfg.sleepIdleMs);

            if (bits & NOTIFY_STOP)      { return; }
            if (bits & NOTIFY_TX_QUEUED) { radio->standby(); _state = State::IDLE; continue; }

            if (bits & NOTIFY_DIO0) {
                size_t pktLen = radio->getPacketLength();
                uint8_t buf[MAX_FRAME_SIZE] = {};
                if (pktLen > 0 && pktLen <= MAX_FRAME_SIZE) {
                    int16_t rxErr = radio->readData(buf, pktLen);
                    if (rxErr == RADIO_ERR_NONE) {
                        float rssi = radio->getRSSI();
                        float snr  = radio->getSNR();
                        dispatchRx(buf, pktLen, rssi, snr);
                    } else {
                        ESP_LOGD(TAG, "readData error %d", rxErr);
                    }
                }
                _state = State::IDLE;
            } else {
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

            uint32_t bits = waitNotify(portMAX_DELAY);

            if (bits & NOTIFY_STOP) { return; }

            if (bits & (NOTIFY_TX_QUEUED | NOTIFY_DIO0)) {
                radio->standby();
                ESP_LOGI(TAG, "Modem wake-up");
                _state = State::IDLE;
            }
        }

    } // while(true)
}
