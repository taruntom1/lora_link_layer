# LoRa Receiver Example

Demonstrates continuous LoRa packet reception using the `lora_radio` component.
The node registers an RX callback that logs every received packet (source ID,
destination ID, sequence number, message type, RSSI, SNR, and payload).

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

The `sdkconfig.defaults` file pre-sets `CONFIG_LORA_NODE_ID=0x0002`. Change it
if you need a different receiver ID.

## Build and flash

```bash
idf.py set-target esp32s3
idf.py menuconfig   # set GPIO pins
idf.py build flash monitor
```

## Expected output

```
I (xxx) lora_receiver: LoRa receiver example starting (node 0x0002)
I (xxx) lora_receiver: Radio initialised. Waiting for packets...
I (xxx) lora_receiver: RX  src=0x0001 dst=0xFFFF seq=0 type=0x01 RSSI=-42.0 dBm SNR=9.5 dB  payload="Hello #0 from 0x0001"
I (xxx) lora_receiver: RX  src=0x0001 dst=0xFFFF seq=1 type=0x01 RSSI=-43.5 dBm SNR=9.2 dB  payload="Hello #1 from 0x0001"
```

## Pairing with the sender example

Flash `examples/sender` (with `CONFIG_LORA_NODE_ID=0x0001`) on a second board.
Both boards must use the same frequency, bandwidth, spreading factor, coding rate,
and sync word — these are all configurable via **Component config → LoRa Radio**
in menuconfig.
