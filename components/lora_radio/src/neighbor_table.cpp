#include "neighbor_table.hpp"

#include "esp_timer.h"

#include <cstring>

NeighborTable::NeighborTable()
{
    std::memset(_entries, 0, sizeof(_entries));
}

void NeighborTable::update(uint16_t nodeId, float rssi, float snr)
{
    uint32_t nowMs = static_cast<uint32_t>(esp_timer_get_time() / 1000ULL);
    int freeSlot = -1;

    for (size_t i = 0; i < CONFIG_LORA_MAX_NEIGHBORS; ++i) {
        if (_entries[i].valid && _entries[i].nodeId == nodeId) {
            _entries[i].rssi       = rssi;
            _entries[i].snr        = snr;
            _entries[i].lastSeenMs = nowMs;
            return;
        }
        if (!_entries[i].valid && freeSlot < 0) {
            freeSlot = static_cast<int>(i);
        }
    }

    if (freeSlot < 0) {
        uint32_t oldest = UINT32_MAX;
        for (size_t i = 0; i < CONFIG_LORA_MAX_NEIGHBORS; ++i) {
            if (_entries[i].lastSeenMs < oldest) {
                oldest   = _entries[i].lastSeenMs;
                freeSlot = static_cast<int>(i);
            }
        }
    }

    _entries[freeSlot] = {nodeId, rssi, snr, nowMs, true};
}

size_t NeighborTable::getAll(NeighborEntry* out, size_t maxCount) const
{
    size_t written = 0;
    for (size_t i = 0; i < CONFIG_LORA_MAX_NEIGHBORS && written < maxCount; ++i) {
        if (_entries[i].valid) {
            out[written++] = _entries[i];
        }
    }
    return written;
}
