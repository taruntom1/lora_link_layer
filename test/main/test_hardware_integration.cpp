// ---------------------------------------------------------------------------
// test_hardware_integration.cpp — Layer 3: on-hardware integration tests
// ---------------------------------------------------------------------------
// These tests require:
//   1. Two ESP32-S3 + SX1278 boards within RF range of each other.
//   2. Matching Kconfig values (frequency, SF, BW, sync word, TX power).
//   3. CONFIG_LORA_HW_TEST_ENABLED=y and LORA_HW_TEST_PEER_NODE_ID set to
//      the partner board's CONFIG_LORA_NODE_ID.
//   4. All GPIO pins configured in sdkconfig (LORA_PIN_SCK/MISO/MOSI/NSS/
//      RST/DIO0/DIO1).
//
// Coordination protocol:
//   Both boards run the same firmware.  Tests that require a specific TX/RX
//   role simply have one board send and the other listen.  Since both devices
//   run the entire suite, out-of-order execution is tolerated by generous
//   wait timeouts.
//
// Build command:
//   cd test && idf.py -DIDF_TARGET=esp32s3 build flash monitor
// ---------------------------------------------------------------------------

#include "unity.h"
#include "lora_radio.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <cstring>

#if CONFIG_LORA_HW_TEST_ENABLED

static const char* TAG     = "test_hw";
static LoraRadio   g_radio;   // shared across hardware tests

static volatile bool     g_rxFired  = false;
static PacketHeader      g_rxHdr    = {};
static uint8_t           g_rxPayload[LORA_MAX_PAYLOAD] = {};
static float             g_rxRssi   = 0.0f;
static float             g_rxSnr    = 0.0f;

static void hwRxCallback(const PacketHeader& hdr,
                          const uint8_t*      payload,
                          float               rssi,
                          float               snr)
{
    g_rxHdr   = hdr;
    std::memcpy(g_rxPayload, payload, hdr.payloadLen < LORA_MAX_PAYLOAD
                                       ? hdr.payloadLen : LORA_MAX_PAYLOAD);
    g_rxRssi  = rssi;
    g_rxSnr   = snr;
    g_rxFired = true;
}

static void resetRxState()
{
    g_rxFired = false;
    std::memset(&g_rxHdr, 0, sizeof(g_rxHdr));
    std::memset(g_rxPayload, 0, sizeof(g_rxPayload));
    g_rxRssi = g_rxSnr = 0.0f;
}

static bool waitForRx(uint32_t timeoutMs = 5000)
{
    uint32_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeoutMs);
    while (xTaskGetTickCount() < deadline) {
        if (g_rxFired) return true;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return g_rxFired;
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

static void test_hw_init_radio(void)
{
    // Initialise the shared radio object once for all HW tests.
    g_radio.setRxCallback(hwRxCallback);
    TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, g_radio.init(),
        "Radio init() failed — check GPIO pin configuration in sdkconfig");
    ESP_LOGI(TAG, "Radio initialised. Node ID: 0x%04X, peer: 0x%04X",
             CONFIG_LORA_NODE_ID, CONFIG_LORA_HW_TEST_PEER_NODE_ID);

    // Brief settle time
    vTaskDelay(pdMS_TO_TICKS(500));
}

static void test_hw_unicast_tx_rx(void)
{
    // Device A sends a unicast to Device B; Device B must receive it.
    // Since both devices run the same test suite, only the smaller node ID
    // acts as transmitter to avoid a collision.
    resetRxState();

    if (CONFIG_LORA_NODE_ID < CONFIG_LORA_HW_TEST_PEER_NODE_ID) {
        // We are Device A — transmit
        uint8_t payload[] = { 0xDE, 0xAD, 0xBE, 0xEF };
        esp_err_t err = g_radio.send(
            CONFIG_LORA_HW_TEST_PEER_NODE_ID, 0x01, payload, sizeof(payload));
        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err, "send() failed for unicast TX");
        ESP_LOGI(TAG, "Unicast sent to 0x%04X", CONFIG_LORA_HW_TEST_PEER_NODE_ID);
        // Allow time for partner to receive and test to pass
        vTaskDelay(pdMS_TO_TICKS(3000));
    } else {
        // We are Device B — wait for reception
        TEST_ASSERT_TRUE_MESSAGE(waitForRx(5000),
            "No unicast packet received within 5 s");
        TEST_ASSERT_EQUAL_HEX16(CONFIG_LORA_HW_TEST_PEER_NODE_ID, g_rxHdr.srcId);
        TEST_ASSERT_EQUAL_HEX16(CONFIG_LORA_NODE_ID,              g_rxHdr.dstId);
        TEST_ASSERT_EQUAL(4u, g_rxHdr.payloadLen);
        TEST_ASSERT_EQUAL_HEX8(0xDE, g_rxPayload[0]);
        TEST_ASSERT_EQUAL_HEX8(0xAD, g_rxPayload[1]);
        TEST_ASSERT_EQUAL_HEX8(0xBE, g_rxPayload[2]);
        TEST_ASSERT_EQUAL_HEX8(0xEF, g_rxPayload[3]);
    }
}

static void test_hw_broadcast_rx(void)
{
    resetRxState();

    if (CONFIG_LORA_NODE_ID < CONFIG_LORA_HW_TEST_PEER_NODE_ID) {
        uint8_t payload[] = { 'H', 'I' };
        esp_err_t err = g_radio.send(0xFFFF, 0x02, payload, sizeof(payload));
        TEST_ASSERT_EQUAL(ESP_OK, err);
        ESP_LOGI(TAG, "Broadcast sent");
        vTaskDelay(pdMS_TO_TICKS(3000));
    } else {
        TEST_ASSERT_TRUE_MESSAGE(waitForRx(5000),
            "No broadcast received within 5 s");
        TEST_ASSERT_EQUAL_HEX16(0xFFFF, g_rxHdr.dstId);
        TEST_ASSERT_EQUAL(2u, g_rxHdr.payloadLen);
    }
}

static void test_hw_ack_roundtrip(void)
{
    // Only the smaller node ID sends with ACK request.
    if (CONFIG_LORA_NODE_ID < CONFIG_LORA_HW_TEST_PEER_NODE_ID) {
        uint8_t payload[] = { 0x42 };
        esp_err_t err = g_radio.send(
            CONFIG_LORA_HW_TEST_PEER_NODE_ID, 0x03, payload, 1, /*ack=*/true);
        TEST_ASSERT_EQUAL_MESSAGE(ESP_OK, err, "send(ack=true) returned error");
        // Allow time for ACK exchange to complete
        vTaskDelay(pdMS_TO_TICKS(3000));
    } else {
        // Partner auto-ACKs; wait for the original message
        TEST_ASSERT_TRUE(waitForRx(5000));
        ESP_LOGI(TAG, "ACK auto-sent for seq=%u", g_rxHdr.seqNum);
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

static void test_hw_rssi_snr_plausibility(void)
{
    // After at least one reception, RSSI and SNR must be in physically
    // plausible ranges for a nearby (<10 m) transmitter.
    if (!g_rxFired && CONFIG_LORA_NODE_ID > CONFIG_LORA_HW_TEST_PEER_NODE_ID) {
        // Ensure we've received something recently
        resetRxState();
        TEST_ASSERT_TRUE_MESSAGE(waitForRx(5000), "No packet received for RSSI check");
    }

    if (g_rxFired) {
        ESP_LOGI(TAG, "Last packet: RSSI=%.1f SNR=%.1f",
                 (double)g_rxRssi, (double)g_rxSnr);
        TEST_ASSERT_GREATER_OR_EQUAL_FLOAT(-130.0f, g_rxRssi);
        TEST_ASSERT_LESS_OR_EQUAL_FLOAT(    -20.0f, g_rxRssi);
        TEST_ASSERT_GREATER_OR_EQUAL_FLOAT( -20.0f, g_rxSnr);
        TEST_ASSERT_LESS_OR_EQUAL_FLOAT(     15.0f, g_rxSnr);
    }
}

static void test_hw_neighbor_table_populated(void)
{
    // After the above exchanges, the peer's ID must appear in the neighbor table.
    NeighborEntry nbrs[CONFIG_LORA_MAX_NEIGHBORS] = {};
    size_t count = g_radio.getNeighbors(nbrs, CONFIG_LORA_MAX_NEIGHBORS);

    bool peerFound = false;
    for (size_t i = 0; i < count; ++i) {
        if (nbrs[i].nodeId == CONFIG_LORA_HW_TEST_PEER_NODE_ID) {
            peerFound = true;
            ESP_LOGI(TAG, "Peer 0x%04X: RSSI=%.1f SNR=%.1f",
                     nbrs[i].nodeId, (double)nbrs[i].rssi, (double)nbrs[i].snr);
            break;
        }
    }
    TEST_ASSERT_TRUE_MESSAGE(peerFound,
        "Peer node not found in neighbor table after packet exchange");
}

static void test_hw_deinit_radio(void)
{
    g_radio.deinit();
    // No assertion — just ensures clean teardown without crash
}

// ---------------------------------------------------------------------------
// Suite entry point (called by test_runner only when HW tests are enabled)
// ---------------------------------------------------------------------------

void run_test_hardware_integration(void)
{
    RUN_TEST(test_hw_init_radio);
    RUN_TEST(test_hw_unicast_tx_rx);
    RUN_TEST(test_hw_broadcast_rx);
    RUN_TEST(test_hw_ack_roundtrip);
    RUN_TEST(test_hw_rssi_snr_plausibility);
    RUN_TEST(test_hw_neighbor_table_populated);
    RUN_TEST(test_hw_deinit_radio);
}

#else // !CONFIG_LORA_HW_TEST_ENABLED

// Stub: when hardware tests are disabled, provide an empty function body so
// the test runner still links correctly.
void run_test_hardware_integration(void) {}

#endif // CONFIG_LORA_HW_TEST_ENABLED
