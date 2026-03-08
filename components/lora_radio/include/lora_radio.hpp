#pragma once

/**
 * @file lora_radio.hpp
 * @brief LoRa Physical / Link Layer component public API for ESP-IDF.
 *
 * @details
 * Provides a FreeRTOS-based LoRa radio driver with:
 *  - Thread-safe transmit queue (LoraRadio::send())
 *  - Automatic CAD (Channel Activity Detection) before every TX
 *  - Optional ACK with configurable retransmission backoff
 *  - ACK result callback (LoraRadio::setAckCallback()) notifies whether an
 *    acknowledged send succeeded or exhausted all retries
 *  - Continuous receive with user-supplied callback (LoraRadio::setRxCallback())
 *  - Neighbour RSSI/SNR table (LoraRadio::getNeighbors())
 *  - Power-saving modem sleep when the channel is idle
 *
 * Architecture overview:
 * @verbatim
 *  ┌─────────────────────────────────────────────────────────────────────┐
 *  │                        Caller (application)                         │
 *  │  LoraRadio::send() / setRxCallback() / setAckCallback()            │
 *  └───────────────────────┬─────────────────────────────────────────────┘
 *                          │ thread-safe queue push + task notify
 *  ┌───────────────────────▼─────────────────────────────────────────────┐
 *  │              FreeRTOS Radio Task (taskLoop)                         │
 *  │  State machine: IDLE → CAD → TX_WAIT → WAIT_ACK → RX → SLEEPING   │
 *  │  All SPI work, IRadioBackend calls, ACK logic live here.            │
 *  └───────────────────────▲─────────────────────────────────────────────┘
 *                          │ xTaskNotifyFromISR (bitmask)
 *  ┌───────────────────────┴─────────────────────────────────────────────┐
 *  │  ISRs: dio0Isr / dio1Isr  (IRAM, zero SPI contact)                 │
 *  │  Only post a notification bit and yield — nothing else.             │
 *  └─────────────────────────────────────────────────────────────────────┘
 * @endverbatim
 *
 * The concrete radio hardware is abstracted behind IRadioBackend.
 * Production builds use Sx1278Backend (wraps RadioLib SX1278).
 * Test builds inject MockRadioBackend via initForTest().
 */

#include <cstdint>
#include <cstddef>
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "ack_tracker.hpp"
#include "lora_config.hpp"
#include "lora_packet.hpp"
#include "neighbor_table.hpp"
#include "radio_backend.hpp"

class EspHal;  // forward-declare to avoid pulling all RadioLib headers in here

class LoraRadio {
public:
    /// @brief Callback invoked (from the radio task) on every received packet.
    /// Must be safe to call from a non-ISR FreeRTOS task context.
    using RxCallback = void (*)(const PacketHeader& header,
                                const uint8_t*      payload,
                                float               rssi,
                                float               snr);

    /// @brief Callback invoked (from the radio task) when an acknowledged send
    /// completes — either successfully or after exhausting all retries.
    ///
    /// @param seqNum  Sequence number of the original transmitted packet.
    /// @param success @c true  if a matching link-layer ACK was received;
    ///                @c false if the maximum retransmission count was reached
    ///                         without receiving an ACK.
    ///
    /// Must be safe to call from a non-ISR FreeRTOS task context.
    using AckCallback = void (*)(uint8_t seqNum, bool success);

    /// @brief Construct a LoraRadio with the given (or default Kconfig) configuration.
    /// @param cfg  Radio and task configuration.  Defaults to Kconfig values.
    explicit LoraRadio(const LoraRadioConfig& cfg = LoraRadioConfig{});

    /// RAII destructor — calls deinit() automatically.
    ~LoraRadio();

    // -----------------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------------

    /**
     * @brief Initialise hardware (SPI, GPIO, RadioLib, FreeRTOS objects).
     *
     * Must be called once from @c app_main before any send or receive
     * operations.  Calling this a second time is a no-op.
     *
     * @return
     *    - ESP_OK on success
     *    - ESP_FAIL if the radio modem could not be initialised
     */
    esp_err_t init();

    /**
     * @brief Gracefully shut down: stop the radio task, put the modem to
     *        sleep, and free the SPI bus.
     *
     * Safe to call even if @c init() was never called or already failed.
     */
    void deinit();

    // -----------------------------------------------------------------------
    // Transmit API  (thread-safe — can be called from any task)
    // -----------------------------------------------------------------------

    /**
     * @brief Queue a packet for transmission.
     *
     * The payload is copied into an internal buffer so the caller may free
     * @p data immediately after this call returns.
     *
     * @param dstId      Destination node ID (0xFFFF for broadcast).
     * @param msgType    Caller-defined message type byte (0x04 is reserved
     *                   for link-layer ACK and must not be used by callers).
     * @param data       Pointer to the payload bytes to transmit.
     * @param len        Payload length in bytes (max @c LORA_MAX_PAYLOAD).
     * @param requestAck If @c true the radio waits for an ACK and
     *                   retransmits up to @c maxRetries times on timeout.
     *
     * @return
     *    - ESP_OK on success
     *    - ESP_ERR_INVALID_SIZE if @p len exceeds @c LORA_MAX_PAYLOAD
     *    - ESP_ERR_NO_MEM if the transmit queue is full
     */
    esp_err_t send(uint16_t        dstId,
                   uint8_t         msgType,
                   const uint8_t*  data,
                   size_t          len,
                   bool            requestAck = false);

    // -----------------------------------------------------------------------
    // Receive
    // -----------------------------------------------------------------------

    /// @brief Register a callback that is invoked for every received packet.
    /// @param cb  Function to call.  Pass @c nullptr to disable.
    void setRxCallback(RxCallback cb);

    /// @brief Register a callback that is invoked when an acknowledged send
    /// completes (ACK received or retries exhausted).
    /// @param cb  Function to call.  Pass @c nullptr to disable.
    void setAckCallback(AckCallback cb);

    // -----------------------------------------------------------------------
    // Neighbour table
    // -----------------------------------------------------------------------

    /**
     * @brief Copy valid entries from the internal neighbour table into @p out.
     *
     * @param out       Output buffer to receive neighbour entries.  Must be
     *                  at least @p maxCount elements long.
     * @param maxCount  Maximum number of entries to copy.
     *
     * @return Number of valid entries written to @p out.
     */
    size_t getNeighbors(NeighborEntry* out, size_t maxCount) const;

    // -----------------------------------------------------------------------
    // Test seam (only available when CONFIG_LORA_ENABLE_TEST_SEAM=y)
    // These methods expose internal state for unit testing.
    // They must NOT be called in production firmware.
    // -----------------------------------------------------------------------

    /// Radio state machine states.
    /// Exposed publicly so tests can observe state without an accessor.
    /// Production callers have no reason to read this value.
    enum class State : uint8_t {
        IDLE,
        CAD,
        TX_WAIT,
        WAIT_ACK,
        RX,
        SLEEPING,
    };

#if CONFIG_LORA_ENABLE_TEST_SEAM
    /// Initialise with an externally supplied (mock) backend instead of a
    /// real SX1278.  No EspHal or Module is created.  The caller retains
    /// ownership of @p backend and must keep it alive until deinit().
    esp_err_t initForTest(IRadioBackend* backend);

    /// Directly invoke the neighbour-table update path (bypasses radio).
    void injectNeighborUpdate(uint16_t nodeId, float rssi, float snr);

    /// Expose the current state-machine state for assertion in tests.
    /// NOTE: Reading this from a different task is inherently racy; always
    /// add a short vTaskDelay before asserting to let the radio task settle.
    State getState() const { return _state; }
#endif

private:
    using TxItem = LoraTxItem;
    static constexpr size_t MAX_FRAME_SIZE = ::MAX_FRAME_SIZE;

    static constexpr uint32_t NOTIFY_DIO0      = (1u << 0);
    static constexpr uint32_t NOTIFY_DIO1      = (1u << 1);
    static constexpr uint32_t NOTIFY_TX_QUEUED = (1u << 2);
    static constexpr uint32_t NOTIFY_STOP      = (1u << 3);
    static constexpr uint32_t NOTIFY_ALL_BITS  = 0xFFFFFFFFu;

    // -----------------------------------------------------------------------
    // Static ISR trampolines
    // -----------------------------------------------------------------------
    static void IRAM_ATTR dio0Isr();
    static void IRAM_ATTR dio1Isr();

    // Static bridge from xTaskCreateStatic C void* arg to C++ instance
    static void taskEntryStatic(void* arg);

    // -----------------------------------------------------------------------
    // Internal helpers (all called from the radio task)
    // -----------------------------------------------------------------------
    void taskLoop();

    /// Shared initialisation called by both init() and initForTest().
    esp_err_t _initCommon(IRadioBackend* backend);

    bool dequeueNextTx(TxItem& out);
    uint32_t waitNotify(uint32_t timeoutMs);
    void dispatchRx(const uint8_t* frame, size_t len, float rssi, float snr);

    bool handleIdle();
    bool handleCad();
    bool handleTxWait();
    bool handleWaitAck();
    bool handleRx();
    bool handleSleeping();

    using StateHandler = bool (LoraRadio::*)();
    static constexpr size_t STATE_HANDLER_COUNT = 6;
    static const StateHandler s_handlers[STATE_HANDLER_COUNT];

    LoraRadioConfig _cfg;
    State           _state  = State::IDLE;
    bool            _initialized = false;

    // Radio backend (injected or created in init)
    IRadioBackend*  _backend     = nullptr;
    bool            _ownsBackend = false;   ///< true → deinit() deletes _backend

    // Raw hardware objects (heap-allocated once in init(), nullptr in test mode)
    EspHal* _hal       = nullptr;
    void*   _module    = nullptr;   // Module*  — kept as void* to avoid pulling
    void*   _sx1278Raw = nullptr;   // SX1278*    RadioLib headers into this .hpp

    // FreeRTOS handles
    TaskHandle_t   _taskHandle  = nullptr;
    QueueHandle_t  _normalQueue = nullptr;

    // Static storage — allocated in BSS section, never on the heap after init()
    static StackType_t   s_taskStack[CONFIG_LORA_TASK_STACK_SIZE];
    static StaticTask_t  s_taskTcb;

    // TX queue storage (depth from Kconfig)
    static uint8_t          s_normalQueueStorage[CONFIG_LORA_TX_QUEUE_DEPTH * sizeof(TxItem)];
    static StaticQueue_t    s_normalQueueState;

    // Neighbour table (static array, size from Kconfig)
    static NeighborEntry    s_neighbors[CONFIG_LORA_MAX_NEIGHBORS];

    // Singleton pointer — used exclusively by ISR trampolines
    static LoraRadio*       s_instance;

    // Guard that prevents xTaskCreateStatic from reusing static TCB/stack
    // buffers while a previous task instance is still executing.
    // Set to true at task entry, cleared to false before vTaskDelete.
    static volatile bool    s_taskRunning;

    // Handle of the currently running task, held in a static so that
    // _initCommon() can send NOTIFY_STOP to a stuck task (e.g., one
    // abandoned by a longjmp in a failed unit test) before waiting.
    static TaskHandle_t     s_runningTaskHandle;

    // Application-layer receive callback
    RxCallback _rxCb = nullptr;

    // Application-layer ACK result callback
    AckCallback _ackCb = nullptr;

    // Rolling TX sequence counter
    uint8_t  _seqNum = 0;

    // CAD retry counter (reset each time a new packet enters CAD phase)
    uint8_t  _cadRetries = 0;
    // Currently staged TX item during CAD / TX_WAIT phases
    TxItem   _currentTx{};
    bool     _hasPendingTx = false;

    NeighborTable _neighbors;
    AckTracker _ackTracker;
};
