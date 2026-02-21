// ---------------------------------------------------------------------------
// test_packet_building.cpp — Layer 1: wire-format constants and structs
// ---------------------------------------------------------------------------
// Tests that do not require hardware, a running task, or a mock radio.
// All assertions can be evaluated at compile or early runtime.
// ---------------------------------------------------------------------------

#include "unity.h"
#include "lora_radio.hpp"
#include <cstring>
#include <cstddef>

// ---------------------------------------------------------------------------
// Suite registration (called from test_runner.cpp)
// ---------------------------------------------------------------------------

static void test_header_size_is_8_bytes(void)
{
    // The wire format is fixed at 8 bytes.  Any change breaks interoperability.
    TEST_ASSERT_EQUAL(8u, PACKET_HEADER_SIZE);
    TEST_ASSERT_EQUAL(8u, sizeof(PacketHeader));
}

static void test_max_payload_is_247(void)
{
    // 255 (SX1278 limit) minus 8 (header) must equal 247.
    TEST_ASSERT_EQUAL(247u, LORA_MAX_PAYLOAD);
}

static void test_header_is_packed_no_padding(void)
{
    // sizeof == sum of member sizes proves no padding was inserted.
    // 2+2+1+1+1+1 = 8
    TEST_ASSERT_EQUAL(
        sizeof(uint16_t) + sizeof(uint16_t) +
        sizeof(uint8_t)  + sizeof(uint8_t) +
        sizeof(uint8_t)  + sizeof(uint8_t),
        sizeof(PacketHeader)
    );
}

static void test_header_field_byte_offsets(void)
{
    // Manually check that each field lands at the expected byte offset in the
    // serialised frame.  This verifies the packed layout is stable across
    // compilers and compiler versions.
    TEST_ASSERT_EQUAL(0u, offsetof(PacketHeader, srcId));
    TEST_ASSERT_EQUAL(2u, offsetof(PacketHeader, dstId));
    TEST_ASSERT_EQUAL(4u, offsetof(PacketHeader, seqNum));
    TEST_ASSERT_EQUAL(5u, offsetof(PacketHeader, msgType));
    TEST_ASSERT_EQUAL(6u, offsetof(PacketHeader, flags));
    TEST_ASSERT_EQUAL(7u, offsetof(PacketHeader, payloadLen));
}

static void test_flag_values_no_overlap(void)
{
    // Each flag bit must be distinct.
    TEST_ASSERT_EQUAL(0x01u, (uint8_t)FLAG_ACK_REQUEST);
    TEST_ASSERT_EQUAL(0x02u, (uint8_t)FLAG_IS_RETRANSMIT);
    TEST_ASSERT_EQUAL(0u, FLAG_ACK_REQUEST & FLAG_IS_RETRANSMIT);
}

static void test_broadcast_address_value(void)
{
    // 0xFFFF is the universal broadcast destination.
    // It must be representable in the 16-bit dstId field.
    const uint16_t broadcast = 0xFFFF;
    TEST_ASSERT_EQUAL(0xFFFF, broadcast);
    // Normal node IDs must differ from broadcast
    const uint16_t normalNode = CONFIG_LORA_NODE_ID;
    TEST_ASSERT_NOT_EQUAL(0xFFFF, normalNode);
}

static void test_header_serialise_round_trip(void)
{
    // Build a header in a raw byte buffer, read it back, verify fields.
    uint8_t frame[8] = {};
    PacketHeader* hdr = reinterpret_cast<PacketHeader*>(frame);
    hdr->srcId      = 0x1234;
    hdr->dstId      = 0xABCD;
    hdr->seqNum     = 0x42;
    hdr->msgType    = 0x01;
    hdr->flags      = FLAG_ACK_REQUEST;
    hdr->payloadLen = 10;

    // Read back through separate pointer (simulates deserialisation)
    const PacketHeader* rx = reinterpret_cast<const PacketHeader*>(frame);
    TEST_ASSERT_EQUAL_HEX16(0x1234, rx->srcId);
    TEST_ASSERT_EQUAL_HEX16(0xABCD, rx->dstId);
    TEST_ASSERT_EQUAL_HEX8(0x42,    rx->seqNum);
    TEST_ASSERT_EQUAL_HEX8(0x01,    rx->msgType);
    TEST_ASSERT_EQUAL_HEX8(FLAG_ACK_REQUEST, rx->flags);
    TEST_ASSERT_EQUAL(10,            rx->payloadLen);
}

static void test_ack_request_flag_combined_with_retransmit(void)
{
    // Both flags should be combinable without collision.
    uint8_t combined = FLAG_ACK_REQUEST | FLAG_IS_RETRANSMIT;
    TEST_ASSERT_TRUE(combined & FLAG_ACK_REQUEST);
    TEST_ASSERT_TRUE(combined & FLAG_IS_RETRANSMIT);
    TEST_ASSERT_EQUAL(0x03u, combined);
}

// ---------------------------------------------------------------------------
// Suite entry point
// ---------------------------------------------------------------------------

void run_test_packet_building(void)
{
    RUN_TEST(test_header_size_is_8_bytes);
    RUN_TEST(test_max_payload_is_247);
    RUN_TEST(test_header_is_packed_no_padding);
    RUN_TEST(test_header_field_byte_offsets);
    RUN_TEST(test_flag_values_no_overlap);
    RUN_TEST(test_broadcast_address_value);
    RUN_TEST(test_header_serialise_round_trip);
    RUN_TEST(test_ack_request_flag_combined_with_retransmit);
}
