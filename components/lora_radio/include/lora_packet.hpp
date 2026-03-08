#pragma once

#include <cstddef>
#include <cstdint>

/// @brief Bitmask flags carried in PacketHeader::flags.
enum PacketFlags : uint8_t {
    FLAG_ACK_REQUEST   = 0x01,  ///< Sender wants an ACK back
    FLAG_IS_RETRANSMIT = 0x02,  ///< This is a retried copy of an earlier packet
};

/// @brief Link-layer ACK message type reserved by this component.
static constexpr uint8_t MSGTYPE_ACK = 0x04;

/// @brief Maximum application payload bytes in one LoRa frame.
/// SX1278 max is 255; we reserve 8 bytes for the header.
static constexpr size_t LORA_MAX_PAYLOAD = 247;

/// @brief Wire-format packet header placed at the start of every LoRa frame.
/// @note  The struct is packed (no padding) so that sizeof(PacketHeader) == 8.
#pragma pack(push, 1)
struct PacketHeader {
    uint16_t srcId;      ///< Source node ID
    uint16_t dstId;      ///< Destination node ID (0xFFFF = broadcast)
    uint8_t  seqNum;     ///< Rolling sequence number (ACK matching / dedup)
    uint8_t  msgType;    ///< Opaque message type — 0x04 is reserved for link-layer ACK
    uint8_t  flags;      ///< Bitmask of PacketFlags
    uint8_t  payloadLen; ///< Number of payload bytes that follow this header
};
#pragma pack(pop)

static constexpr size_t PACKET_HEADER_SIZE = sizeof(PacketHeader); // = 8
static constexpr size_t MAX_FRAME_SIZE = PACKET_HEADER_SIZE + LORA_MAX_PAYLOAD;

/// @brief Internal queue item used by the radio state machine.
struct LoraTxItem {
    uint8_t  buf[MAX_FRAME_SIZE];
    uint16_t len;          ///< total frame bytes (header + payload)
    uint8_t  retries;      ///< how many times this item has been retried
    bool     needsAck;     ///< should we wait for an ACK?
};

/// @brief Build a header-only ACK frame that mirrors the received sequence number.
inline void buildAckFrame(uint8_t* frameBuf,
                          uint16_t& frameLen,
                          const PacketHeader& rxHdr,
                          uint16_t myNodeId)
{
    PacketHeader* ack = reinterpret_cast<PacketHeader*>(frameBuf);
    ack->srcId      = myNodeId;
    ack->dstId      = rxHdr.srcId;
    ack->seqNum     = rxHdr.seqNum;
    ack->msgType    = MSGTYPE_ACK;
    ack->flags      = 0;
    ack->payloadLen = 0;
    frameLen        = static_cast<uint16_t>(PACKET_HEADER_SIZE);
}
