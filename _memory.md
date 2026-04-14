# Link Layer Memory

## What this layer does in plain language

The repository implements a LoRa link-layer component (`components/lora_radio`) for ESP-IDF. It provides packet framing, medium access control (CAD before transmit), asynchronous transmit/receive handling through a FreeRTOS task, optional link-layer acknowledgments with retries, and neighbor signal-quality tracking. It abstracts hardware-specific operations behind a backend interface so the core logic can be tested without real radios.

## Key components and modules

- **Public API and orchestrator**
  - `components/lora_radio/include/lora_radio.hpp`
  - `components/lora_radio/src/lora_radio.cpp`
  - `components/lora_radio/src/lora_state_handlers.cpp`
  - Main class: `LoraRadio`
  - Owns the radio state machine and FreeRTOS objects.

- **Packet definitions**
  - `components/lora_radio/include/lora_packet.hpp`
  - Defines `PacketHeader` (8-byte packed header), payload/frame limits, flags, ACK message type (`MSGTYPE_ACK = 0x04`), and `buildAckFrame(...)`.

- **ACK tracking**
  - `components/lora_radio/include/ack_tracker.hpp`
  - `components/lora_radio/src/ack_tracker.cpp`
  - Class `AckTracker` manages one pending ACK-required frame, ACK matching, retry count update, retransmit-flag insertion, and failed sequence retrieval.

- **Neighbor table**
  - `components/lora_radio/include/neighbor_table.hpp`
  - `components/lora_radio/src/neighbor_table.cpp`
  - Class `NeighborTable` stores latest RSSI/SNR per node and evicts oldest entry when full.

- **Hardware abstraction**
  - `components/lora_radio/include/radio_backend.hpp` (`IRadioBackend`)
  - `components/lora_radio/include/sx1278_backend.hpp` (`Sx1278Backend`)
  - `components/lora_radio/include/esp_hal_s3.hpp`
  - `components/lora_radio/src/esp_hal_s3.cpp`
  - `IRadioBackend` decouples state machine logic from RadioLib/SX1278.
  - `EspHal` implements RadioLib HAL using ESP-IDF SPI/GPIO/timer APIs.

- **Configuration**
  - `components/lora_radio/include/lora_config.hpp`
  - `components/lora_radio/Kconfig`
  - Runtime config mirrors Kconfig values: RF params, task priorities, queue sizes, retry/timeout values, sleep thresholds, pins, and test seam toggles.

- **Tests**
  - `test/main/test_packet_building.cpp`
  - `test/main/test_send_validation.cpp`
  - `test/main/test_neighbor_table.cpp`
  - `test/main/test_state_machine.cpp`
  - `test/main/test_hardware_integration.cpp`
  - `test/main/mock/mock_radio_backend.hpp`
  - Tests validate wire format, send() validation/sequence behavior, state transitions, ACK flows, auto-ACK, sleep/wake, and neighbor updates.

## How data flows through this layer

1. **Upper-layer enqueue**  
   `LoraRadio::send(...)` constructs `PacketHeader + payload` into `LoraTxItem`, sets flags and sequence number, and pushes it into a static FreeRTOS queue (`_normalQueue`).

2. **Task-driven MAC and transmit path**  
   `taskLoop()` dispatches state handlers:
   - `IDLE`: dequeue frame or start receive.
   - `CAD`: perform channel activity detection; retry/backoff on busy.
   - `TX_WAIT`: wait for TX done (DIO0), then either finish or enter ACK wait.
   - `WAIT_ACK`: wait for ACK frame, retry if timeout, eventually succeed/fail callback.

3. **Receive and dispatch path**  
   In `RX` and `WAIT_ACK`, upon DIO0:
   - Read frame, RSSI, SNR via backend.
   - `dispatchRx(...)` updates neighbor table.
   - Filter by destination (`nodeId` or broadcast).
   - Ignore pure ACK frames for app callback.
   - If incoming frame requests ACK, enqueue ACK frame.
   - Deliver payload to registered `RxCallback`.

4. **Power management path**  
   If RX remains idle for `sleepIdleMs`, state transitions to `SLEEPING` and modem sleeps; `send()` notification wakes it to standby and returns flow to IDLE/CAD.

## How this layer connects upward and downward

- **Upward (network/application layer boundary)**  
  Exposed through `LoraRadio` public API:
  - `init()`, `deinit()`
  - `send(dstId, msgType, data, len, requestAck)`
  - `setRxCallback(...)`
  - `setAckCallback(...)`
  - `getNeighbors(...)`
  
  There is no direct network-layer code in this repository; integration point is this API contract.

- **Downward (radio hardware boundary)**  
  `LoraRadio` talks only to `IRadioBackend`. Production wiring:
  - `EspHal` (ESP-IDF SPI/GPIO/timing HAL) → `Module`/`SX1278` (RadioLib) → `Sx1278Backend` adapter.
  
  DIO interrupts are connected through backend actions and mapped to FreeRTOS task notifications (`NOTIFY_DIO0`, `NOTIFY_DIO1`) by minimal ISR trampolines.

## Important logic identified

- **State machine**
  - Defined by `LoraRadio::State` and handler table `s_handlers` in `lora_state_handlers.cpp`.
  - States: `IDLE`, `CAD`, `TX_WAIT`, `WAIT_ACK`, `RX`, `SLEEPING`.

- **MAC behavior**
  - CAD before transmit (`startChannelScan()`), with retry count `cadRetries` and random backoff.
  - Drop packet after CAD retries exhausted.

- **Addressing and frame semantics**
  - 16-bit node addressing.
  - `dstId=0xFFFF` is broadcast.
  - `msgType=0x04` reserved for link-layer ACK.
  - Flags include `FLAG_ACK_REQUEST` and `FLAG_IS_RETRANSMIT`.

- **Reliability**
  - Optional ACK request per packet.
  - `AckTracker` tracks pending packet and retries up to `maxRetries`.
  - On retry, retransmit flag is set.
  - Success/failure surfaced via `AckCallback`.

- **Concurrency model**
  - `send()` is thread-safe queue producer.
  - Radio task is single consumer and sole executor of backend operations.
  - ISR handlers do not touch SPI; they only notify task.
  - FreeRTOS task/queue use static storage.

- **Neighbor intelligence**
  - Every received frame updates source node RSSI/SNR.
  - Bounded table with oldest-entry replacement when full.

- **Error handling**
  - `send()` validates size and queue capacity (`ESP_ERR_INVALID_SIZE`, `ESP_ERR_NO_MEM`).
  - Init failure path cleans allocated resources.
  - Timeouts in CAD/TX/ACK paths cause fallback behavior (retry, drop, standby/reset to IDLE).

- **Power management**
  - Auto-sleep after RX idle timeout.
  - Wake on TX-queued notification and return to active state.

- **Testing strategy**
  - Test seam (`CONFIG_LORA_ENABLE_TEST_SEAM`) exposes `initForTest`, `injectNeighborUpdate`, and `getState`.
  - Mock backend enables deterministic state-machine tests without hardware.
  - Optional two-node hardware integration tests guarded by `CONFIG_LORA_HW_TEST_ENABLED`.

## Notes and observations for architecture writing

- The code and docs use “link layer” terminology, but class comments sometimes mention “Physical / Link Layer”; architecture write-up should clarify it provides link-layer services over LoRa PHY via RadioLib.
- The architecture is intentionally event-driven: all transitions are driven by queue availability, task notifications, and timeout expiry.
- ACKs are implemented as regular frames with reserved `msgType`, not a separate control channel.
- There is no explicit routing logic in this repository; this should be stated to avoid confusion with network-layer responsibilities.
- Major design seam for testability is `IRadioBackend`; this is central to explain maintainability and portability.
