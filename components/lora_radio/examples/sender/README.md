# Sender example

Broadcasts a text payload every 5 seconds using `lora_radio`.

## Build

```bash
idf.py set-target esp32s3
idf.py menuconfig
idf.py build flash monitor
```

Set pin mappings in `Component config -> LoRa Radio`.