# Copilot Instructions — lora_link_layer

## Project overview
ESP-IDF project (target: **ESP32-S3**) that implements a LoRa V2V/V2X link layer.  
The primary deliverable is the reusable **`lora_radio` component**; `main/` is a thin demo app.

## Repository layout
```
components/lora_radio/   — reusable link-layer component (all logic lives here)
  include/               — public API (lora_radio.hpp, radio_backend.hpp, sx1278_backend.hpp, esp_hal_s3.hpp)
  src/                   — lora_radio.cpp (state machine), esp_hal_s3.cpp (SPI HAL)
  Kconfig                — all tunable params (pins, RF params, task knobs, timeouts)
main/                    — app entry point (main.cpp); only instantiates LoraRadio
test/                    — separate ESP-IDF project for on-target Unity tests
managed_components/      — IDF Component Manager cache (jgromes__radiolib)
```

## Architecture: key design decisions

### Dependency injection via `IRadioBackend`
`lora_radio.cpp` never includes `<RadioLib.h>` directly in the state-machine code. All hardware calls go through `IRadioBackend` (pure virtual). This keeps RadioLib headers out of the state machine and enables testing without hardware.

- **Production**: `init()` heap-allocates `EspHal → Module → SX1278 → Sx1278Backend` and stores the pointer as `IRadioBackend* _backend`.
- **Testing**: `initForTest(IRadioBackend* backend)` injects `MockRadioBackend` (defined in `test/main/mock/`).

### FreeRTOS radio task & ISR contract
The state machine runs in a single FreeRTOS task (`taskLoop`). ISRs (`dio0Isr` / `dio1Isr`, both `IRAM_ATTR`) do **only** `xTaskNotifyFromISR` with a bitmask bit (`NOTIFY_DIO0`, `NOTIFY_DIO1`) — no SPI, no logic. All SPI work happens inside `taskLoop`.

State flow: `IDLE → CAD → TX_WAIT → WAIT_ACK → RX → SLEEPING`

### Static FreeRTOS allocation
All FreeRTOS objects use static storage (`xTaskCreateStatic` / `xQueueCreateStatic`) to avoid `pvPortMalloc` after init. Storage arrays live in BSS (`s_taskStack`, `s_taskTcb`, `s_normalQueueStorage`, `s_neighbors`).

### `EspHal`: portable SPI HAL
The upstream RadioLib `EspHal.h` uses raw register access (`spi_dev_t`/`DPORT_*`) valid only on ESP32 classic. `components/lora_radio/include/esp_hal_s3.hpp` replaces it with the portable `esp-idf spi_master` driver API, required for ESP32-S3/S2/C3/C6.

## Packet format
Wire header is a **packed 8-byte struct** (`PacketHeader`, `#pragma pack(push,1)`):
`srcId(2) | dstId(2) | seqNum(1) | msgType(1) | flags(1) | payloadLen(1)`

- Broadcast: `dstId = 0xFFFF`
- `msgType 0x04` is **reserved** for link-layer ACK — callers must not use it
- Max payload: `LORA_MAX_PAYLOAD = 247` bytes

## Configuration (Kconfig)
All parameters are in `components/lora_radio/Kconfig`. `main/Kconfig` is intentionally empty. All `LoraRadioConfig` fields default to Kconfig values so `LoraRadioConfig{}` is a valid board-independent config. Override individual fields at runtime or via `sdkconfig`.

Key symbols: `CONFIG_LORA_PIN_*`, `CONFIG_LORA_FREQUENCY_HZ`, `CONFIG_LORA_SPREADING_FACTOR`, `CONFIG_LORA_ACK_TIMEOUT_MS`, `CONFIG_LORA_ENABLE_TEST_SEAM`.

## Building & flashing

Use the **ESP-IDF VS Code extension** commands (Build, Flash, Monitor).

- **Main app**: open root folder, build/flash normally.
- **Test binary**: set workspace root to `test/` (or set `IDF_PATH` and run `idf.py` there). The `test/CMakeLists.txt` points `EXTRA_COMPONENT_DIRS` at the parent project's `components/` and `managed_components/` to avoid re-downloading RadioLib.

A `.devcontainer` is provided (ESP-IDF + QEMU image).

## Testing
Tests use the **Unity framework** on-target (no host-side simulation). All test suites run automatically on boot from `test_runner.cpp::app_main`.

```cpp
// Typical test pattern:
MockRadioBackend mock;
LoraRadio radio;
radio.initForTest(&mock);          // inject mock, start FreeRTOS task
// set mock return values, call radio.send(), fire simulated interrupts:
mock.fireDio0();                    // simulates TX-done / RX-done / CAD-clear
mock.fireDio1();                    // simulates CAD-busy
waitForState(radio, LoraRadio::State::RX);
TEST_ASSERT_EQUAL(1, mock.startReceiveCount);
radio.deinit();
```

- **Test seam gate**: `CONFIG_LORA_ENABLE_TEST_SEAM=y` (set in `test/sdkconfig.defaults`) exposes `initForTest()`, `getState()`, `injectNeighborUpdate()`. Never call these in production builds.
- Timing: radio task runs at priority 4 (`CONFIG_LORA_TASK_PRIORITY`), test task at priority 5. `waitForCount()` / `waitForState()` poll in 5 ms steps with a 500 ms budget.
- Hardware integration tests guarded by `CONFIG_LORA_HW_TEST_ENABLED`; real GPIO pins must be uncommented in `test/sdkconfig.defaults`.

## External dependency
RadioLib (`jgromes/radiolib >=7.6.0`) is managed by the IDF Component Manager, declared in `components/lora_radio/idf_component.yml`. Do not add it to `main/idf_component.yml` — it is already transitively available.
