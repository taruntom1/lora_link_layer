#include "ack_tracker.hpp"

#include <cstddef>

void AckTracker::clear()
{
    _pending = {};
    _waiting = false;
}

void AckTracker::startTracking(const LoraTxItem& item)
{
    _pending = item;
    _waiting = true;
}

bool AckTracker::isWaiting() const
{
    return _waiting;
}

bool AckTracker::matchesAck(const PacketHeader& rxHdr, uint16_t myNodeId) const
{
    if (!_waiting) {
        return false;
    }

    const PacketHeader* txHdr = pendingHeader();
    if (!txHdr) {
        return false;
    }

    return (rxHdr.msgType == MSGTYPE_ACK)
        && (rxHdr.seqNum == txHdr->seqNum)
        && (rxHdr.dstId == myNodeId);
}

bool AckTracker::shouldRetry(uint32_t maxRetries)
{
    if (!_waiting || _pending.retries >= maxRetries) {
        return false;
    }

    ++_pending.retries;
    PacketHeader* hdr = pendingHeader();
    if (hdr) {
        hdr->flags |= FLAG_IS_RETRANSMIT;
    }
    return true;
}

const LoraTxItem& AckTracker::pending() const
{
    return _pending;
}

uint8_t AckTracker::failedSeqNum() const
{
    const PacketHeader* hdr = pendingHeader();
    return hdr ? hdr->seqNum : 0;
}

const PacketHeader* AckTracker::pendingHeader() const
{
    if (_pending.len < PACKET_HEADER_SIZE) {
        return nullptr;
    }
    return reinterpret_cast<const PacketHeader*>(_pending.buf);
}

PacketHeader* AckTracker::pendingHeader()
{
    return const_cast<PacketHeader*>(
        static_cast<const AckTracker*>(this)->pendingHeader());
}
