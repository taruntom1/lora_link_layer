# lora_radio

LoRa link-layer component for ESP-IDF using SX1278 (RadioLib backend).

## Features

- FreeRTOS task-based radio state machine (`IDLE -> CAD -> TX_WAIT -> WAIT_ACK -> RX -> SLEEPING`)
- Channel Activity Detection (CAD) before TX
- Optional link-layer ACK with retries and timeout
- Broadcast and unicast packet handling
- Neighbor RSSI/SNR tracking
- Test seam support (`initForTest`, `getState`, `injectNeighborUpdate`) behind `CONFIG_LORA_ENABLE_TEST_SEAM`

## Add as dependency

```yaml
dependencies:
  taruntom1/lora_radio: "*"
```

## Notes

- `msgType = 0x04` is reserved for internal ACK packets.
- Maximum payload is `247` bytes (`LORA_MAX_PAYLOAD`).
- Configure pins and radio settings in `Component config -> LoRa Radio`.

See [README.rst](README.rst) for detailed API and architecture docs.