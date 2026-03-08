#pragma once

#include <cstdint>

#include "sdkconfig.h"

/**
 * @brief Runtime configuration for LoraRadio.
 *
 * All fields carry Kconfig defaults, so @c LoraRadioConfig{} is a valid,
 * board-independent configuration. Override individual members to adapt to
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
