// ---------------------------------------------------------------------------
// test_runner.cpp — Unity test entry point
// ---------------------------------------------------------------------------
// All test suite registration functions are declared here and driven from
// app_main.  Each suite lives in its own translation unit so they can be
// built selectively via menuconfig or compiled together for a full run.
//
// Flash the test binary and observe output on the UART monitor.
// By default all suites are run automatically on boot.
// ---------------------------------------------------------------------------

#include "unity.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "TestRunner";

// ---------------------------------------------------------------------------
// Forward declarations — each suite registers its own tests in these fns
// ---------------------------------------------------------------------------
void run_test_packet_building();
void run_test_send_validation();
void run_test_neighbor_table();
void run_test_state_machine();
void run_test_hardware_integration();

// ---------------------------------------------------------------------------
// app_main — entry point
// ---------------------------------------------------------------------------

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "=== LoRa Radio Unit Test Suite ===");

    UNITY_BEGIN();

    run_test_packet_building();
    run_test_send_validation();
    run_test_neighbor_table();
    run_test_state_machine();

#if CONFIG_LORA_HW_TEST_ENABLED
    run_test_hardware_integration();
#endif

    int failures = UNITY_END();

    if (failures == 0) {
        ESP_LOGI(TAG, "All tests PASSED");
    } else {
        ESP_LOGE(TAG, "%d test(s) FAILED", failures);
    }

    // Prevent watchdog from killing the idle task while the UART drains
    vTaskDelay(pdMS_TO_TICKS(1000));
}
