\mainpage LoRa Radio Component

# LoRa Radio Component

## Overview

The `lora_radio` component provides a FreeRTOS-based LoRa V2V/V2X link-layer driver for ESP32-S3
(and other ESP32 variants) using the [RadioLib](https://github.com/jgromes/RadioLib) SX1278 driver.

A single \ref LoraRadio instance manages one SX1278 transceiver over SPI. All radio hardware access
is abstracted behind the \ref IRadioBackend pure-virtual interface, which makes it possible to inject
a mock backend for unit-testing without any real hardware.

---

## Features

- **Thread-safe transmit queue** – call \ref LoraRadio::send from any FreeRTOS task.
- **Channel Activity Detection (CAD)** – every outgoing frame waits for a clear channel before
  transmitting, with configurable retry-and-backoff.
- **Optional ACK with retransmission** – set `requestAck = true` in \ref LoraRadio::send to enable
  link-layer acknowledged delivery with configurable timeout and retry count.
- **ACK result callback** – register a callback with \ref LoraRadio::setAckCallback to be notified
  whether an acknowledged send completed successfully or exhausted all retries.
- **Continuous receive with callback** – register a callback with \ref LoraRadio::setRxCallback to
  be notified of every received packet.
- **Neighbour RSSI/SNR table** – the component maintains a per-node signal-quality table updated on
  every received frame, readable via \ref LoraRadio::getNeighbors.
- **Modem power-save sleep** – the radio enters low-power sleep automatically after a configurable
  idle period and wakes on the next transmit request.
- **Static FreeRTOS allocation** – all FreeRTOS objects (task, queue) use static storage so that no
  heap allocation occurs after initialization.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                        Caller (application)                         │
│  LoraRadio::send() / setRxCallback() / setAckCallback()            │
└───────────────────────┬─────────────────────────────────────────────┘
                        │ thread-safe queue push + task notify
┌───────────────────────▼─────────────────────────────────────────────┐
│              FreeRTOS Radio Task (taskLoop)                         │
│  State machine: IDLE → CAD → TX_WAIT → WAIT_ACK → RX → SLEEPING   │
│  All SPI work, IRadioBackend calls, ACK logic live here.            │
└───────────────────────▲─────────────────────────────────────────────┘
                        │ xTaskNotifyFromISR (bitmask)
┌───────────────────────┴─────────────────────────────────────────────┐
│  ISRs: dio0Isr / dio1Isr  (IRAM, zero SPI contact)                 │
│  Only post a notification bit and yield — nothing else.             │
└─────────────────────────────────────────────────────────────────────┘
```

### State-machine states

| State | Description |
|-------|-------------|
| **IDLE** | Decides whether to dequeue a TX item (→ CAD) or enter receive mode (→ RX). |
| **CAD** | Performs Channel Activity Detection before each transmission. |
| **TX_WAIT** | Waits for DIO0 (TX-done interrupt) after starting a transmission. |
| **WAIT_ACK** | Waits for a link-layer ACK after an acknowledged transmission; fires the ACK callback on success or when retries are exhausted. |
| **RX** | Radio in continuous receive; fires the RX callback on each packet. |
| **SLEEPING** | Radio in low-power modem sleep; woken by a transmit request. |

---

## Wire Packet Format

Each frame starts with an 8-byte packed header (\ref PacketHeader) followed by up to
`LORA_MAX_PAYLOAD` (247) bytes of application payload.

| Field | Size | Description |
|-------|------|-------------|
| `srcId` | 2 B | Source node ID. |
| `dstId` | 2 B | Destination node ID; `0xFFFF` = broadcast. |
| `seqNum` | 1 B | Rolling sequence number used for ACK matching and deduplication. |
| `msgType` | 1 B | Caller-defined message type; `0x04` is **reserved** for link-layer ACK. |
| `flags` | 1 B | Bitmask of \ref PacketFlags (`FLAG_ACK_REQUEST`, `FLAG_IS_RETRANSMIT`). |
| `payloadLen` | 1 B | Number of payload bytes that follow this header. |

---

## Configuration

All tunable parameters are exposed as Kconfig symbols in `components/lora_radio/Kconfig` and are
mirrored as fields of \ref LoraRadioConfig. Using the default-constructed `LoraRadioConfig{}`
selects all Kconfig values so the driver is board-independent without any code changes.

| Kconfig symbol | Description |
|----------------|-------------|
| `CONFIG_LORA_PIN_*` | SPI and control GPIO assignments. |
| `CONFIG_LORA_FREQUENCY_HZ` | RF centre frequency in Hz. |
| `CONFIG_LORA_SPREADING_FACTOR` | LoRa spreading factor (6–12). |
| `CONFIG_LORA_ACK_TIMEOUT_MS` | Timeout waiting for a link-layer ACK. |
| `CONFIG_LORA_ENABLE_TEST_SEAM` | Exposes `initForTest()`, `getState()`, and `injectNeighborUpdate()` for unit testing. Set to `y` only in test builds. |

---

## Dependencies

RadioLib (`jgromes/radiolib >= 7.6.0`) is declared in `components/lora_radio/idf_component.yml`
and fetched automatically by the IDF Component Manager. Do not add it to `main/idf_component.yml`;
it is already transitively available.

---

## Usage Example

```cpp
#include "lora_radio.hpp"

static void rx_handler(const PacketHeader& hdr,
                        const uint8_t*      payload,
                        float               rssi,
                        float               snr)
{
    ESP_LOGI("app", "RX from 0x%04X  RSSI=%.1f SNR=%.1f", hdr.srcId, rssi, snr);
}

static void ack_handler(uint8_t seqNum, bool success)
{
    if (success) {
        ESP_LOGI("app", "ACK received for seq=%u", seqNum);
    } else {
        ESP_LOGW("app", "No ACK for seq=%u (retries exhausted)", seqNum);
    }
}

void app_main(void)
{
    LoraRadio radio;                   // uses Kconfig defaults
    radio.setRxCallback(rx_handler);
    radio.setAckCallback(ack_handler);
    ESP_ERROR_CHECK(radio.init());

    const uint8_t msg[] = "hello";
    // requestAck=true: ACK result is reported via ack_handler
    radio.send(0xFFFF, 0x01, msg, sizeof(msg) - 1, /*requestAck=*/true);
}
```
