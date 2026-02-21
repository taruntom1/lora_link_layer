// ---------------------------------------------------------------------------
// test_neighbor_table.cpp — Layer 1: neighbour table management tests
// ---------------------------------------------------------------------------
// Uses the CONFIG_LORA_ENABLE_TEST_SEAM helper injectNeighborUpdate() to
// drive neighbour table logic without requiring packet reception over the air.
// ---------------------------------------------------------------------------

#include "unity.h"
#include "lora_radio.hpp"
#include "mock_radio_backend.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

// ---------------------------------------------------------------------------
// Fixture
// ---------------------------------------------------------------------------
struct NbrFixture {
    MockRadioBackend mock;
    LoraRadio        radio;

    NbrFixture() { mock.reset(); }

    void init()
    {
        TEST_ASSERT_EQUAL(ESP_OK, radio.initForTest(&mock));
    }

    ~NbrFixture() { radio.deinit(); }

    void inject(uint16_t id, float rssi = -80.0f, float snr = 7.0f)
    {
        radio.injectNeighborUpdate(id, rssi, snr);
    }

    size_t getAll(NeighborEntry* buf, size_t sz)
    {
        return radio.getNeighbors(buf, sz);
    }
};

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

static void test_single_neighbor_added(void)
{
    NbrFixture f;
    f.init();

    f.inject(0x1234, -75.0f, 9.5f);

    NeighborEntry out[4] = {};
    size_t count = f.getAll(out, 4);
    TEST_ASSERT_EQUAL(1u, count);
    TEST_ASSERT_EQUAL_HEX16(0x1234, out[0].nodeId);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -75.0f, out[0].rssi);
    TEST_ASSERT_FLOAT_WITHIN(0.01f,   9.5f, out[0].snr);
    TEST_ASSERT_TRUE(out[0].valid);
}

static void test_neighbor_updated_not_duplicated(void)
{
    NbrFixture f;
    f.init();

    f.inject(0xAAAA, -60.0f, 5.0f);
    f.inject(0xAAAA, -50.0f, 8.0f);  // same node, updated signal

    NeighborEntry out[8] = {};
    size_t count = f.getAll(out, 8);
    TEST_ASSERT_EQUAL(1u, count);  // must not create two entries
    TEST_ASSERT_FLOAT_WITHIN(0.01f, -50.0f, out[0].rssi);
    TEST_ASSERT_FLOAT_WITHIN(0.01f,   8.0f, out[0].snr);
}

static void test_multiple_distinct_neighbors(void)
{
    NbrFixture f;
    f.init();

    f.inject(0x0001, -70.0f, 6.0f);
    f.inject(0x0002, -72.0f, 5.5f);
    f.inject(0x0003, -68.0f, 7.0f);

    NeighborEntry out[8] = {};
    size_t count = f.getAll(out, 8);
    TEST_ASSERT_EQUAL(3u, count);
}

static void test_get_neighbors_respects_max_count(void)
{
    NbrFixture f;
    f.init();

    for (uint16_t i = 1; i <= 5; ++i) {
        f.inject(i, -70.0f, 7.0f);
    }

    NeighborEntry out[3] = {};
    size_t count = f.getAll(out, 3);
    TEST_ASSERT_EQUAL(3u, count);  // capped at maxCount=3
}

static void test_neighbor_table_full_lru_eviction(void)
{
    NbrFixture f;
    f.init();

    // Fill all CONFIG_LORA_MAX_NEIGHBORS slots.
    // Inject node 0x0001 first so it is the "oldest".
    f.inject(0x0001, -90.0f, 3.0f);
    vTaskDelay(pdMS_TO_TICKS(2));   // ensure distinct lastSeenMs for 0x0001

    for (uint16_t i = 2; i <= CONFIG_LORA_MAX_NEIGHBORS; ++i) {
        f.inject(i, -80.0f, 7.0f);
        vTaskDelay(pdMS_TO_TICKS(1));   // stagger timestamps
    }

    // Table is now full.  Add one more new node — must evict the oldest (0x0001).
    f.inject(0xBEEF, -55.0f, 10.0f);

    NeighborEntry out[CONFIG_LORA_MAX_NEIGHBORS + 1] = {};
    size_t count = f.getAll(out, CONFIG_LORA_MAX_NEIGHBORS + 1);

    // Table size must not exceed CONFIG_LORA_MAX_NEIGHBORS
    TEST_ASSERT_LESS_OR_EQUAL(CONFIG_LORA_MAX_NEIGHBORS, count);

    // The new node must be present
    bool foundNew = false;
    bool foundOld = false;
    for (size_t i = 0; i < count; ++i) {
        if (out[i].nodeId == 0xBEEF) foundNew = true;
        if (out[i].nodeId == 0x0001) foundOld = true;
    }
    TEST_ASSERT_TRUE_MESSAGE(foundNew, "New node 0xBEEF not found after LRU eviction");
    TEST_ASSERT_FALSE_MESSAGE(foundOld, "Old node 0x0001 was not evicted (LRU failure)");
}

static void test_get_neighbors_returns_zero_when_empty(void)
{
    NbrFixture f;
    f.init();

    NeighborEntry out[4] = {};
    size_t count = f.getAll(out, 4);
    TEST_ASSERT_EQUAL(0u, count);
}

static void test_neighbor_rssi_and_snr_stored_correctly(void)
{
    NbrFixture f;
    f.init();

    f.inject(0x5678, -112.3f, -5.7f);

    NeighborEntry out[2] = {};
    size_t count = f.getAll(out, 2);
    TEST_ASSERT_EQUAL(1u, count);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -112.3f, out[0].rssi);
    TEST_ASSERT_FLOAT_WITHIN(0.1f,   -5.7f, out[0].snr);
}

// ---------------------------------------------------------------------------
// Suite entry point
// ---------------------------------------------------------------------------

void run_test_neighbor_table(void)
{
    RUN_TEST(test_single_neighbor_added);
    RUN_TEST(test_neighbor_updated_not_duplicated);
    RUN_TEST(test_multiple_distinct_neighbors);
    RUN_TEST(test_get_neighbors_respects_max_count);
    RUN_TEST(test_neighbor_table_full_lru_eviction);
    RUN_TEST(test_get_neighbors_returns_zero_when_empty);
    RUN_TEST(test_neighbor_rssi_and_snr_stored_correctly);
}
