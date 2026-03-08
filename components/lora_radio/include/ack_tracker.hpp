#pragma once

#include "lora_packet.hpp"

class AckTracker {
public:
    void clear();

    void startTracking(const LoraTxItem& item);
    bool isWaiting() const;

    bool matchesAck(const PacketHeader& rxHdr, uint16_t myNodeId) const;
    bool shouldRetry(uint32_t maxRetries);

    const LoraTxItem& pending() const;
    uint8_t failedSeqNum() const;

private:
    const PacketHeader* pendingHeader() const;
    PacketHeader* pendingHeader();

    LoraTxItem _pending{};
    bool _waiting = false;
};
