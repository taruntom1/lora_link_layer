# Copilot Instructions — lora_link_layer

## Big picture
- ESP-IDF project (ESP32-S3 target) delivering a reusable LoRa link-layer component: `components/lora_radio`.
- `main/src/main.cpp` is intentionally thin (demo wiring only); keep protocol/driver logic in `components/lora_radio`.
- `test/` is a separate ESP-IDF project that reuses parent `components/` + `managed_components/` via `test/CMakeLists.txt`.

## Core architecture patterns
- Preserve backend abstraction: `lora_radio.cpp` runs against `IRadioBackend` (`include/radio_backend.hpp`), not raw RadioLib APIs.
- Production path (`LoraRadio::init`) constructs `EspHal -> Module -> SX1278 -> Sx1278Backend`; test path (`initForTest`) injects `MockRadioBackend`.
- Keep ISR contract strict: `dio0Isr`/`dio1Isr` only notify task bits (`xTaskNotifyFromISR`), no SPI/business logic in ISR context.
- State machine flow to maintain: `IDLE -> CAD -> TX_WAIT -> WAIT_ACK -> RX -> SLEEPING` (see `src/lora_radio.cpp`).
- FreeRTOS objects are statically allocated (`xTaskCreateStatic`, `xQueueCreateStatic` + `s_taskStack`, `s_taskTcb`, queue storage); avoid heap allocation after init except explicit radio object creation in init path.
- Only one live `LoraRadio` instance is supported (singleton ISR trampoline pattern in `lora_radio.hpp`).

## Protocol and data contracts
- Wire header is packed `PacketHeader` (8 bytes): `srcId|dstId|seqNum|msgType|flags|payloadLen` in `include/lora_radio.hpp`.
- `msgType=0x04` is reserved for link-layer ACK (`MSGTYPE_ACK` in `src/lora_radio.cpp`); application messages must use other values.
- Broadcast destination is `0xFFFF`; max payload is `LORA_MAX_PAYLOAD=247` bytes.

## Configuration conventions
- Tunables live in `components/lora_radio/Kconfig`; `LoraRadioConfig` defaults mirror Kconfig so `LoraRadioConfig{}` is a valid default config.
- `main/Kconfig` is intentionally empty; do not move LoRa config there.
- High-impact symbols: `CONFIG_LORA_PIN_*`, `CONFIG_LORA_ACK_TIMEOUT_MS`, `CONFIG_LORA_CAD_RETRIES`, `CONFIG_LORA_ENABLE_TEST_SEAM`, `CONFIG_LORA_HW_TEST_ENABLED`.

## Build, test, and docs workflows
- Use ESP-IDF VS Code extension commands for firmware (`Build`, `Flash`, `Monitor`).
- Main firmware: operate from repo root.
- Test firmware: open/run from `test/` project; all Unity suites run from `test/main/test_runner.cpp::app_main`.
- Unit tests are on-target (not host simulation); typical pattern is `MockRadioBackend + initForTest() + fireDio0()/fireDio1()`.
- Test seam APIs (`initForTest`, `getState`, `injectNeighborUpdate`) are gated by `CONFIG_LORA_ENABLE_TEST_SEAM` and must remain test-only.
- Docs pipeline: `docs/README.md` + `tools/build-docs.ps1`; VS Code task `Build Docs` executes the script.

## Dependencies and integration points
- RadioLib dependency is declared in `components/lora_radio/idf_component.yml` (`jgromes/radiolib >=7.6.0`).
- Do not duplicate RadioLib dependency in `main/idf_component.yml`; `main` only pins ESP-IDF version.
- `EspHal` (`include/esp_hal_s3.hpp`, `src/esp_hal_s3.cpp`) exists because upstream RadioLib `EspHal` is not suitable for ESP32-S3; keep this portability layer intact when touching SPI/HAL code.
