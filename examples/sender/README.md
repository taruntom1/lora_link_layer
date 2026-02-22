# LoRa Sender Example

Demonstrates periodic LoRa broadcast using the `lora_radio` component. The node
broadcasts a short text message every 5 seconds to all nodes on the network
(`dstId = 0xFFFF`).

## Hardware required

- ESP32-S3 development board
- SX1278 (or compatible) LoRa module

## Pin configuration

All GPIO assignments **must** be configured before building. Open menuconfig and
navigate to **Component config → LoRa Radio**:

| Parameter | menuconfig symbol | Notes |
|-----------|-------------------|-------|
| SPI SCK   | `LORA_PIN_SCK`    | SPI clock |
| SPI MISO  | `LORA_PIN_MISO`   | SPI data in |
| SPI MOSI  | `LORA_PIN_MOSI`   | SPI data out |
| NSS / CS  | `LORA_PIN_NSS`    | Chip-select |
| RST       | `LORA_PIN_RST`    | Hardware reset |
| DIO0      | `LORA_PIN_DIO0`   | TX/RX-done interrupt |
| DIO1      | `LORA_PIN_DIO1`   | CAD-busy interrupt |

The `sdkconfig.defaults` file pre-sets `CONFIG_LORA_NODE_ID=0x0001`. Change it
if you need a different sender ID.

## Build and flash

```bash
idf.py set-target esp32s3
idf.py menuconfig   # set GPIO pins
idf.py build flash monitor
```

## Expected output

```
I (xxx) lora_sender: LoRa sender example starting (node 0x0001)
I (xxx) lora_sender: Radio initialised. Broadcasting every 5000 ms.
I (xxx) lora_sender: TX queued: "Hello #0 from 0x0001"
I (xxx) lora_sender: TX queued: "Hello #1 from 0x0001"
```

## Pairing with the receiver example

Flash `examples/receiver` (with `CONFIG_LORA_NODE_ID=0x0002`) on a second board
with the same frequency and spreading-factor settings. The receiver will log each
packet as it arrives.
