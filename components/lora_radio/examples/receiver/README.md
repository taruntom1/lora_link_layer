# Receiver example

Continuously receives packets and logs source, destination, sequence, RSSI, SNR, and payload.

## Build

```bash
idf.py set-target esp32s3
idf.py menuconfig
idf.py build flash monitor
```

Set pin mappings in `Component config -> LoRa Radio`.