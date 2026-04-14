# Link Layer Architecture Plan

## 1) Introduction and Scope
- **Coverage:** Define responsibilities of this repository’s Link Layer and explicit boundaries with upper and lower layers.
- **Code basis:** `components/lora_radio/include/lora_radio.hpp`, `docs/mainpage.md`, `components/lora_radio/README.rst`.

## 2) Repository and Component Structure
- **Coverage:** Explain project layout, core component, and test organization.
- **Code basis:** root `CMakeLists.txt`, `components/lora_radio/CMakeLists.txt`, `test/main/CMakeLists.txt`, file tree under `components/lora_radio` and `test/main`.

## 3) External Interfaces and Integration Boundaries
- **Coverage:** Public API (`init`, `deinit`, `send`, callbacks, neighbor readout), how upper layers interact, and how hardware is abstracted.
- **Code basis:** `components/lora_radio/include/lora_radio.hpp`, `components/lora_radio/include/radio_backend.hpp`, `components/lora_radio/include/sx1278_backend.hpp`.

## 4) Configuration Model
- **Coverage:** Runtime config object and Kconfig symbols controlling RF, pins, timing, reliability, and test toggles.
- **Code basis:** `components/lora_radio/include/lora_config.hpp`, `components/lora_radio/Kconfig`.

## 5) Packet Model and Frame Semantics
- **Coverage:** `PacketHeader` layout, size constraints, broadcast semantics, reserved ACK message type, and control flags.
- **Code basis:** `components/lora_radio/include/lora_packet.hpp`, `test/main/test_packet_building.cpp`.

## 6) Runtime Concurrency and Execution Model
- **Coverage:** FreeRTOS task ownership model, static task/queue allocation, queue producer/consumer behavior, ISR notification design.
- **Code basis:** `components/lora_radio/src/lora_radio.cpp`, `components/lora_radio/include/lora_radio.hpp`.

## 7) State Machine Architecture
- **Coverage:** State set, state handler table, transition triggers and timeout-driven behavior.
- **Code basis:** `components/lora_radio/src/lora_state_handlers.cpp`, `components/lora_radio/include/lora_radio.hpp`, `test/main/test_state_machine.cpp`.

## 8) Transmission and MAC Workflow
- **Coverage:** TX enqueue, CAD clear/busy handling, random backoff, transmit completion handling.
- **Code basis:** `components/lora_radio/src/lora_radio.cpp`, `components/lora_radio/src/lora_state_handlers.cpp`.

## 9) ACK Reliability Workflow
- **Coverage:** ACK request semantics, matching rules, retry policy, retransmit flag behavior, callback outcomes.
- **Code basis:** `components/lora_radio/src/lora_state_handlers.cpp`, `components/lora_radio/src/ack_tracker.cpp`, `components/lora_radio/include/ack_tracker.hpp`, `test/main/test_state_machine.cpp`.

## 10) Receive Pipeline and Inbound Filtering
- **Coverage:** RX data read path, destination filtering, ACK suppression from app callback, auto-ACK generation, callback dispatch.
- **Code basis:** `components/lora_radio/src/lora_state_handlers.cpp`, `components/lora_radio/include/lora_packet.hpp`.

## 11) Neighbor Tracking Subsystem
- **Coverage:** Entry lifecycle, update/replace policy, returned data model, relationship with RX/ACK processing.
- **Code basis:** `components/lora_radio/src/neighbor_table.cpp`, `components/lora_radio/include/neighbor_table.hpp`, relevant calls in `components/lora_radio/src/lora_state_handlers.cpp`.

## 12) Power Management Behavior
- **Coverage:** RX idle timeout transition to sleep and wake-up path on queued TX.
- **Code basis:** `components/lora_radio/src/lora_state_handlers.cpp`, `components/lora_radio/include/lora_config.hpp`, `components/lora_radio/Kconfig`.

## 13) Hardware Abstraction and Platform Layer
- **Coverage:** `IRadioBackend` contract and concrete SX1278 adapter, `EspHal` SPI/GPIO/interrupt/timing implementation.
- **Code basis:** `components/lora_radio/include/radio_backend.hpp`, `components/lora_radio/include/sx1278_backend.hpp`, `components/lora_radio/include/esp_hal_s3.hpp`, `components/lora_radio/src/esp_hal_s3.cpp`.

## 14) Initialization and Shutdown Lifecycle
- **Coverage:** Production init path, test init path, resource ownership, teardown sequence, cleanup on failures.
- **Code basis:** `components/lora_radio/src/lora_radio.cpp`.

## 15) Testability and Verification Architecture
- **Coverage:** Test seam design, mock backend approach, and what each test suite validates.
- **Code basis:** `components/lora_radio/include/lora_radio.hpp`, `test/main/mock/mock_radio_backend.hpp`, all files under `test/main`.

## Planned Diagrams

1. **Layer context and integration boundary diagram**  
   - **Section:** 3 (External Interfaces and Integration Boundaries)  
   - **Purpose:** Show upper-layer caller boundary, `LoraRadio` core, backend abstraction, and SX1278 hardware path.

2. **State machine diagram**  
   - **Section:** 7 (State Machine Architecture)  
   - **Purpose:** Show `IDLE`, `CAD`, `TX_WAIT`, `WAIT_ACK`, `RX`, `SLEEPING` and main transition triggers.

3. **Transmit + CAD + ACK sequence diagram**  
   - **Section:** 9 (ACK Reliability Workflow)  
   - **Purpose:** Show enqueue, CAD, TX done, ACK wait, retry loop, and callback outcomes.

4. **Receive processing flowchart**  
   - **Section:** 10 (Receive Pipeline and Inbound Filtering)  
   - **Purpose:** Show frame parse/filter, auto-ACK condition, callback dispatch, and neighbor update.

5. **Initialization/teardown ownership diagram**  
   - **Section:** 14 (Initialization and Shutdown Lifecycle)  
   - **Purpose:** Show object creation in production mode vs test mode and shutdown cleanup order.

6. **Neighbor table update/eviction flowchart**  
   - **Section:** 11 (Neighbor Tracking Subsystem)  
   - **Purpose:** Show update existing vs insert new vs evict oldest when full.
