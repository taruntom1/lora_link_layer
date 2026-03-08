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

#include <cstring>

static const char* TAG = "LoraRadio";

// ===========================================================================
// Static storage — allocated in BSS section, never on the heap after init()
// ===========================================================================

StackType_t  LoraRadio::s_taskStack[CONFIG_LORA_TASK_STACK_SIZE];
StaticTask_t LoraRadio::s_taskTcb;

uint8_t      LoraRadio::s_normalQueueStorage[
    CONFIG_LORA_TX_QUEUE_DEPTH * sizeof(LoraRadio::TxItem)];
StaticQueue_t LoraRadio::s_normalQueueState;

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
        ESP_LOGW(TAG, "init() called twice - ignoring");
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
             static_cast<double>(_cfg.frequencyMHz),
             _cfg.spreadingFactor,
             static_cast<double>(_cfg.bandwidthKHz),
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
        _cfg.txPowerDbm);

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
        ESP_LOGW(TAG, "DIO1 not connected - CAD busy detection disabled");
    }

    // ------------------------------------------------------------------
    // Create statically-allocated FreeRTOS TX queue.
    // ------------------------------------------------------------------
    _normalQueue = xQueueCreateStatic(
        CONFIG_LORA_TX_QUEUE_DEPTH,
        sizeof(TxItem),
        s_normalQueueStorage,
        &s_normalQueueState);

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
        &s_taskTcb);

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
    _ackTracker.clear();
    ESP_LOGI(TAG, "Radio deinitialized");
}

// ===========================================================================
// Test seam  (compiled only when CONFIG_LORA_ENABLE_TEST_SEAM=y)
// ===========================================================================

#if CONFIG_LORA_ENABLE_TEST_SEAM

esp_err_t LoraRadio::initForTest(IRadioBackend* backend)
{
    if (_initialized) {
        ESP_LOGW(TAG, "initForTest() called twice - ignoring");
        return ESP_OK;
    }
    // In test mode: no EspHal, Module, or SX1278 are created.
    // The backend is externally owned; deinit() must not delete it.
    _ownsBackend = false;
    return _initCommon(backend);
}

void LoraRadio::injectNeighborUpdate(uint16_t nodeId, float rssi, float snr)
{
    _neighbors.update(nodeId, rssi, snr);
}

#endif // CONFIG_LORA_ENABLE_TEST_SEAM

// ===========================================================================
// Public TX API  (thread-safe — callable from any task)
// ===========================================================================

esp_err_t LoraRadio::send(uint16_t       dstId,
                           uint8_t       msgType,
                          const uint8_t* data,
                           size_t        len,
                           bool          requestAck)
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

void LoraRadio::setAckCallback(AckCallback cb)
{
    _ackCb = cb;
}

size_t LoraRadio::getNeighbors(NeighborEntry* out, size_t maxCount) const
{
    return _neighbors.getAll(out, maxCount);
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
    s_taskRunning = true;
    s_runningTaskHandle = self->_taskHandle;
    self->taskLoop();
    s_runningTaskHandle = nullptr;
    s_taskRunning = false;
    vTaskDelete(nullptr);
}

// ===========================================================================
// Helper: wait for task notifications with a timeout
// ===========================================================================

uint32_t LoraRadio::waitNotify(uint32_t timeoutMs)
{
    uint32_t bits = 0;
    xTaskNotifyWait(0, NOTIFY_ALL_BITS, &bits, pdMS_TO_TICKS(timeoutMs));
    return bits;
}

// ===========================================================================
// Helper: dequeue the next TX item
// ===========================================================================

bool LoraRadio::dequeueNextTx(TxItem& out)
{
    return xQueueReceive(_normalQueue, &out, 0) == pdTRUE;
}

void LoraRadio::taskLoop()
{
    ESP_LOGI(TAG, "State machine started");
    _state = State::IDLE;

    while (true) {
        const size_t stateIndex = static_cast<size_t>(_state);
        if (stateIndex >= STATE_HANDLER_COUNT) {
            ESP_LOGE(TAG, "invalid state index %u", static_cast<unsigned>(stateIndex));
            return;
        }

        if (!(this->*s_handlers[stateIndex])()) {
            return;
        }
    }
}
