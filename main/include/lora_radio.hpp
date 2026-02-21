#pragma once

// ---------------------------------------------------------------------------
// lora_radio.hpp — LoRa Physical / Link Layer Component for ESP-IDF
// ---------------------------------------------------------------------------
// Architecture overview:
//
//  ┌─────────────────────────────────────────────────────────────────────┐
//  │                        Caller (application)                         │
//  │  LoraRadio::send() / setRxCallback()                               │
//  └───────────────────────┬─────────────────────────────────────────────┘
//                          │ thread-safe queue push + task notify
//  ┌───────────────────────▼─────────────────────────────────────────────┐
//  │              FreeRTOS Radio Task (taskLoop)                         │
//  │  State machine: IDLE → CAD → TX_WAIT → WAIT_ACK → RX → SLEEPING   │
//  │  All SPI work, RadioLib calls, ACK logic live here.                 │
//  └───────────────────────▲─────────────────────────────────────────────┘
//                          │ xTaskNotifyFromISR (bitmask)
//  ┌───────────────────────┴─────────────────────────────────────────────┐
//  │  ISRs: dio0Isr / dio1Isr  (IRAM, zero SPI contact)                 │
//  │  Only post a notification bit and yield — nothing else.             │
//  └─────────────────────────────────────────────────────────────────────┘
// ---------------------------------------------------------------------------

#include <cstdint>
#include <cstddef>

#include "sdkconfig.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ============================================================================
// Wire-format packet header (packed — no padding)
// Sits at the front of every LoRa frame transmitted by this component.
// ============================================================================

/// Flags carried in PacketHeader::flags
enum PacketFlags : uint8_t {
    FLAG_ACK_REQUEST   = 0x01,  ///< Sender wants an ACK back
    FLAG_IS_RETRANSMIT = 0x02,  ///< This is a retried copy of an earlier packet
};

/// Maximum application payload bytes in one LoRa frame.
/// SX1278 max is 255; we reserve 8 bytes for the header.
static constexpr size_t LORA_MAX_PAYLOAD = 247;

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

/// Per-node entry updated on every received packet
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
struct LoraRadioConfig {
    // SPI bus
    int  spiHost  = CONFIG_LORA_SPI_HOST;
    int  pinSck   = CONFIG_LORA_PIN_SCK;
    int  pinMiso  = CONFIG_LORA_PIN_MISO;
    int  pinMosi  = CONFIG_LORA_PIN_MOSI;
    // Radio control pins
    int  pinNss   = CONFIG_LORA_PIN_NSS;
    int  pinRst   = CONFIG_LORA_PIN_RST;
    int  pinDio0  = CONFIG_LORA_PIN_DIO0;
    int  pinDio1  = CONFIG_LORA_PIN_DIO1;
    // RF parameters
    float   frequencyMHz    = CONFIG_LORA_FREQUENCY_HZ / 1e6f;
    float   bandwidthKHz    = CONFIG_LORA_BANDWIDTH_HZ / 1e3f;
    uint8_t spreadingFactor = CONFIG_LORA_SPREADING_FACTOR;
    uint8_t codingRate      = CONFIG_LORA_CODING_RATE;
    int8_t  txPowerDbm      = CONFIG_LORA_TX_POWER_DBM;
    uint8_t syncWord        = CONFIG_LORA_SYNC_WORD;
    // Node identity
    uint16_t nodeId         = CONFIG_LORA_NODE_ID;
    // FreeRTOS task knobs
    uint32_t taskStackSize  = CONFIG_LORA_TASK_STACK_SIZE;
    uint32_t taskPriority   = CONFIG_LORA_TASK_PRIORITY;
    uint32_t txQueueDepth   = CONFIG_LORA_TX_QUEUE_DEPTH;
    // Reliability knobs
    uint32_t maxNeighbors   = CONFIG_LORA_MAX_NEIGHBORS;
    uint32_t maxRetries     = CONFIG_LORA_MAX_RETRIES;
    uint32_t ackTimeoutMs   = CONFIG_LORA_ACK_TIMEOUT_MS;
    uint32_t sleepIdleMs    = CONFIG_LORA_SLEEP_IDLE_MS;
    uint32_t cadRetries     = CONFIG_LORA_CAD_RETRIES;
};

// ============================================================================
// LoraRadio class
// ============================================================================

class EspHal;  // forward-declare to avoid pulling all RadioLib headers in here

class LoraRadio {
public:
    /// Callback invoked (from the radio task) on every received packet.
    /// Must be safe to call from a non-ISR FreeRTOS task context.
    using RxCallback = void (*)(const PacketHeader& header,
                                const uint8_t*      payload,
                                float               rssi,
                                float               snr);

    explicit LoraRadio(const LoraRadioConfig& cfg = LoraRadioConfig{});

    /// RAII destructor — calls deinit() automatically.
    ~LoraRadio();

    // -----------------------------------------------------------------------
    // Lifecycle
    // -----------------------------------------------------------------------

    /// Initialise hardware (SPI, GPIO, RadioLib, FreeRTOS objects).
    /// Must be called once from app_main before any send/receive operations.
    esp_err_t init();

    /// Gracefully shut down: stop task, put radio to sleep, free SPI bus.
    void deinit();

    // -----------------------------------------------------------------------
    // Transmit API  (thread-safe — can be called from any task)
    // -----------------------------------------------------------------------

    /// Queue a packet for transmission.
    /// @param dstId      Destination node ID (0xFFFF for broadcast)
    /// @param msgType    Caller-defined message type byte (0x04 is reserved for ACK)
    /// @param data       Payload bytes (copied into internal buffer)
    /// @param len        Payload length  (max LORA_MAX_PAYLOAD bytes)
    /// @param requestAck If true, the radio will wait for an ACK and
    ///                   retransmit up to maxRetries times on timeout.
    /// @return ESP_OK | ESP_ERR_INVALID_SIZE | ESP_ERR_NO_MEM (queue full)
    esp_err_t send(uint16_t        dstId,
                   uint8_t         msgType,
                   const uint8_t*  data,
                   size_t          len,
                   bool            requestAck = false);

    // -----------------------------------------------------------------------
    // Receive
    // -----------------------------------------------------------------------
    void setRxCallback(RxCallback cb);

    // -----------------------------------------------------------------------
    // Neighbour table
    // -----------------------------------------------------------------------

    /// Copy up to maxCount entries from the internal neighbour table into
    /// @p out.  Returns the number of valid entries written.
    size_t getNeighbors(NeighborEntry* out, size_t maxCount) const;

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
    // State machine
    // -----------------------------------------------------------------------
    enum class State : uint8_t {
        IDLE,
        CAD,
        TX_WAIT,
        WAIT_ACK,
        RX,
        SLEEPING,
    };

    // -----------------------------------------------------------------------
    // Task-notification bit definitions
    // DIO0 and DIO1 ISRs use xTaskNotifyFromISR with eSetBits so that the
    // task can distinguish which pin fired without polling.
    // NOTIFY_TX_QUEUED wakes the task from SLEEPING when a new packet arrives.
    // -----------------------------------------------------------------------
    static constexpr uint32_t NOTIFY_DIO0      = (1u << 0);
    static constexpr uint32_t NOTIFY_DIO1      = (1u << 1);
    static constexpr uint32_t NOTIFY_TX_QUEUED = (1u << 2);
    static constexpr uint32_t NOTIFY_STOP      = (1u << 3);
    static constexpr uint32_t NOTIFY_ALL_BITS  = 0xFFFFFFFFu;

    // -----------------------------------------------------------------------
    // Static ISR trampolines
    // RadioLib's attachInterrupt API accepts void(*)(void) — no context arg.
    // The static instance pointer s_instance is the only way to reach the
    // task handle from inside the ISR without modifying RadioLib internals.
    // Marked IRAM_ATTR: must be in fast IRAM to be callable during flash ops.
    // -----------------------------------------------------------------------
    static void IRAM_ATTR dio0Isr();  ///< TX/RX done OR CAD no-activity
    static void IRAM_ATTR dio1Isr();  ///< CAD preamble detected (busy)

    // Static bridge from xTaskCreateStatic's C  void*  arg to C++ instance
    static void taskEntryStatic(void* arg);

    // -----------------------------------------------------------------------
    // Internal helpers (all called from the radio task)
    // -----------------------------------------------------------------------
    void taskLoop();

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

    // RadioLib objects (heap-allocated once in init(), freed in deinit())
    EspHal* _hal    = nullptr;
    void*   _module = nullptr;   // Module*  — kept as void* to avoid pulling
    void*   _radio  = nullptr;   // SX1278*    RadioLib headers into this .hpp

    // FreeRTOS handles
    TaskHandle_t   _taskHandle = nullptr;
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

    // Application-layer receive callback (set by setRxCallback)
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
