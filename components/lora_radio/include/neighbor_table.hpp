#pragma once

#include <cstddef>
#include <cstdint>

#include "sdkconfig.h"

/// @brief Per-node entry updated on every received packet.
struct NeighborEntry {
    uint16_t nodeId;       ///< Source node ID
    float    rssi;         ///< Last measured RSSI (dBm)
    float    snr;          ///< Last measured SNR (dB)
    uint32_t lastSeenMs;   ///< esp_timer millis when packet was received
    bool     valid;        ///< true if the slot holds a real entry
};

class NeighborTable {
public:
    NeighborTable();

    void update(uint16_t nodeId, float rssi, float snr);
    size_t getAll(NeighborEntry* out, size_t maxCount) const;

private:
    NeighborEntry _entries[CONFIG_LORA_MAX_NEIGHBORS];
};
