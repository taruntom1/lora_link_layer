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
 *  - Continuous receive with user-supplied callback (LoraRadio::setRxCallback())
 *  - Neighbour RSSI/SNR table (LoraRadio::getNeighbors())
 *  - Power-saving modem sleep when the channel is idle
 *
 * Architecture overview:
 * @verbatim
 *  ┌─────────────────────────────────────────────────────────────────────┐
 *  │                        Caller (application)                         │
 *  │  LoraRadio::send() / setRxCallback()                               │
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

#include "sdkconfig.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "radio_backend.hpp"

// ============================================================================
// Wire-format packet header (packed — no padding)
// Sits at the front of every LoRa frame transmitted by this component.
// ============================================================================

/// @brief Bitmask flags carried in PacketHeader::flags.
enum PacketFlags : uint8_t {
    FLAG_ACK_REQUEST   = 0x01,  ///< Sender wants an ACK back
    FLAG_IS_RETRANSMIT = 0x02,  ///< This is a retried copy of an earlier packet
};

/// @brief Maximum application payload bytes in one LoRa frame.
/// SX1278 max is 255; we reserve 8 bytes for the header.
static constexpr size_t LORA_MAX_PAYLOAD = 247;

/// @brief Wire-format packet header placed at the start of every LoRa frame.
/// @note  The struct is packed (no padding) so that sizeof(PacketHeader) == 8.
#pragma pack(push, 1)
struct PacketHeader {
    uint16_t srcId;      ///< Source node ID
    uint16_t dstId;      ///< Destination node ID (0xFFFF = broadcast)
    uint8_t  seqNum;     ///< Rolling sequence number (ACK matching / dedup)
    uint8_t  msgType;    ///< Opaque message type — defined by the caller; 0x04 is reserved for link-layer ACK
    uint8_t  flags;      ///< Bitmask of PacketFlags
    uint8_t  payloadLen; ///< Number of payload bytes that follow this header
};
#pragma pack(pop)

static constexpr size_t PACKET_HEADER_SIZE = sizeof(PacketHeader); // = 8

// ============================================================================
// Neighbor table
// ============================================================================

/// @brief Per-node entry updated on every received packet.
struct NeighborEntry {
    uint16_t nodeId;       ///< Source node ID
    float    rssi;         ///< Last measured RSSI (dBm)
    float    snr;          ///< Last measured SNR (dB)
    uint32_t lastSeenMs;   ///< esp_timer millis when packet was received
    bool     valid;        ///< true if the slot holds a real entry
};

// ============================================================================
// Configuration struct
// All fields default to values from Kconfig so callers can use {} for a fully
// board-independent config or override individual pins at runtime.
// ============================================================================

/**
 * @brief Runtime configuration for LoraRadio.
 *
 * All fields carry Kconfig defaults, so @c LoraRadioConfig{} is a valid,
 * board-independent configuration.  Override individual members to adapt to
 * a specific board without touching @c sdkconfig.
 */
struct LoraRadioConfig {
    // SPI bus
    int  spiHost  = CONFIG_LORA_SPI_HOST;  ///< ESP-IDF SPI host identifier (e.g. @c SPI2_HOST)
    int  pinSck   = CONFIG_LORA_PIN_SCK;   ///< GPIO number for the SPI clock line
    int  pinMiso  = CONFIG_LORA_PIN_MISO;  ///< GPIO number for the MISO line
    int  pinMosi  = CONFIG_LORA_PIN_MOSI;  ///< GPIO number for the MOSI line
    // Radio control pins
    int  pinNss   = CONFIG_LORA_PIN_NSS;   ///< GPIO number for chip-select (NSS / CS)
    int  pinRst   = CONFIG_LORA_PIN_RST;   ///< GPIO number for hardware reset
    int  pinDio0  = CONFIG_LORA_PIN_DIO0;  ///< GPIO number for DIO0 interrupt (TX-done / RX-done / CAD-clear)
    int  pinDio1  = CONFIG_LORA_PIN_DIO1;  ///< GPIO number for DIO1 interrupt (CAD-busy); use @c RADIO_PIN_NC if not connected
    // RF parameters
    float   frequencyMHz    = CONFIG_LORA_FREQUENCY_HZ / 1e6f;  ///< Centre frequency in MHz
    float   bandwidthKHz    = CONFIG_LORA_BANDWIDTH_HZ / 1e3f;  ///< Signal bandwidth in kHz
    uint8_t spreadingFactor = CONFIG_LORA_SPREADING_FACTOR;     ///< LoRa spreading factor (6–12)
    uint8_t codingRate      = CONFIG_LORA_CODING_RATE;          ///< Coding-rate denominator (5–8, meaning 4/5 … 4/8)
    int8_t  txPowerDbm      = CONFIG_LORA_TX_POWER_DBM;         ///< TX output power in dBm
    uint8_t syncWord        = CONFIG_LORA_SYNC_WORD;            ///< 1-byte LoRa sync word (0x12 = private, 0x34 = LoRaWAN)
    // Node identity
    uint16_t nodeId         = CONFIG_LORA_NODE_ID;              ///< This node's 16-bit network identifier
    // FreeRTOS task knobs
    uint32_t taskStackSize  = CONFIG_LORA_TASK_STACK_SIZE;      ///< Radio task stack size in bytes
    uint32_t taskPriority   = CONFIG_LORA_TASK_PRIORITY;        ///< Radio task FreeRTOS priority
    uint32_t txQueueDepth   = CONFIG_LORA_TX_QUEUE_DEPTH;       ///< Depth of the transmit queue (in TxItem units)
    // Reliability knobs
    uint32_t maxNeighbors   = CONFIG_LORA_MAX_NEIGHBORS;        ///< Maximum number of neighbour table entries
    uint32_t maxRetries     = CONFIG_LORA_MAX_RETRIES;          ///< Maximum ACK retransmission attempts
    uint32_t ackTimeoutMs   = CONFIG_LORA_ACK_TIMEOUT_MS;       ///< Timeout waiting for a link-layer ACK (ms)
    uint32_t sleepIdleMs    = CONFIG_LORA_SLEEP_IDLE_MS;        ///< Idle time before entering modem sleep (ms)
    uint32_t cadRetries     = CONFIG_LORA_CAD_RETRIES;          ///< Maximum CAD retries before dropping a packet
};

// ============================================================================
// LoraRadio class
// ============================================================================

/**
 * @brief FreeRTOS-based LoRa link-layer driver.
 *
 * A single instance manages one SX1278 radio.  The public API is thread-safe;
 * all SPI work and state-machine logic run inside the dedicated radio task.
 *
 * @note Only one LoraRadio instance may exist at a time because the ISR
 *       trampolines (dio0Isr / dio1Isr) use a static singleton pointer.
 */
class EspHal;  // forward-declare to avoid pulling all RadioLib headers in here

class LoraRadio {
public:
    /// @brief Callback invoked (from the radio task) on every received packet.
    /// Must be safe to call from a non-ISR FreeRTOS task context.
    using RxCallback = void (*)(const PacketHeader& header,
                                const uint8_t*      payload,
                                float               rssi,
                                float               snr);

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
    // -----------------------------------------------------------------------
    // Internal packet container used by both TX queues
    // -----------------------------------------------------------------------
    static constexpr size_t MAX_FRAME_SIZE = PACKET_HEADER_SIZE + LORA_MAX_PAYLOAD;

    struct TxItem {
        uint8_t  buf[MAX_FRAME_SIZE];
        uint16_t len;          ///< total frame bytes (header + payload)
        uint8_t  retries;      ///< how many times this item has been retried
        bool     needsAck;     ///< should we wait for an ACK?
    };

    // -----------------------------------------------------------------------
    // Task-notification bit definitions
    // -----------------------------------------------------------------------
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

    // Neighbour table management
    void updateNeighbor(uint16_t nodeId, float rssi, float snr);

    // Packet helpers
    bool     dequeueNextTx(TxItem& out);
    void     buildAck(uint8_t* frameBuf, uint16_t& frameLen,
                      const PacketHeader& rxHdr);
    void     dispatchRx(const uint8_t* frame, size_t len, float rssi, float snr);
    uint32_t waitNotify(uint32_t timeoutMs);

    // -----------------------------------------------------------------------
    // Data members
    // -----------------------------------------------------------------------
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

    // Rolling TX sequence counter
    uint8_t  _seqNum = 0;

    // Pending ACK tracking
    TxItem   _pendingAck{};
    bool     _waitingAck = false;

    // CAD retry counter (reset each time a new packet enters CAD phase)
    uint8_t  _cadRetries = 0;

    // Currently staged TX item during CAD / TX_WAIT phases
    TxItem   _currentTx{};
    bool     _hasPendingTx = false;
};
