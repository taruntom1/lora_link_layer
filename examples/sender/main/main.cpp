// ---------------------------------------------------------------------------
// sender/main/main.cpp — LoRa Sender Example
// ---------------------------------------------------------------------------
// Demonstrates how to use the lora_radio component to periodically broadcast
// a simple message over LoRa.  Configure GPIO pins via menuconfig before
// flashing (Component config → LoRa Radio).
// ---------------------------------------------------------------------------

#include <cstdio>
#include "lora_radio.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "lora_sender";

// Application message type for user data (0x04 is reserved for link-layer ACK)
static constexpr uint8_t  MSG_TYPE_DATA  = 0x01;
// Send to all nodes on the network
static constexpr uint16_t BROADCAST_ADDR = 0xFFFF;
// Transmit interval
static constexpr uint32_t TX_INTERVAL_MS = 5000;

static LoraRadio radio;  // Config defaults from Kconfig / sdkconfig.defaults

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "LoRa sender example starting (node 0x%04X)",
             (unsigned)CONFIG_LORA_NODE_ID);

    esp_err_t err = radio.init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "radio.init() failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "Radio initialised. Broadcasting every %lu ms.",
             (unsigned long)TX_INTERVAL_MS);

    uint32_t counter = 0;
    while (true) {
        char    msg[32];
        int     len = snprintf(msg, sizeof(msg), "Hello #%lu from 0x%04X",
                               (unsigned long)counter,
                               (unsigned)CONFIG_LORA_NODE_ID);

        err = radio.send(BROADCAST_ADDR, MSG_TYPE_DATA,
                         reinterpret_cast<const uint8_t *>(msg),
                         static_cast<size_t>(len));
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "TX queued: \"%s\"", msg);
        } else {
            ESP_LOGW(TAG, "send() failed: %s", esp_err_to_name(err));
        }

        ++counter;
        vTaskDelay(pdMS_TO_TICKS(TX_INTERVAL_MS));
    }
}
