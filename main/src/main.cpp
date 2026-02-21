// ---------------------------------------------------------------------------
// main.cpp — Application entry point
// ---------------------------------------------------------------------------
// Instantiates LoraRadio using pin assignments from sdkconfig and starts
// the radio state machine.  Extend this file to add application logic.
// ---------------------------------------------------------------------------

#include "lora_radio.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "app_main";

static LoraRadio radio;  // Config defaults from Kconfig

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "LoRa link-layer demo starting");

    esp_err_t err = radio.init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "radio.init() failed: %s", esp_err_to_name(err));
        return;
    }

    // Register a simple receive callback
    radio.setRxCallback([](const PacketHeader& hdr,
                           const uint8_t*      payload,
                           float               rssi,
                           float               snr)
    {
        ESP_LOGI(TAG, "RX: src=0x%04X dst=0x%04X seq=%u type=%u len=%u "
                 "RSSI=%.1f SNR=%.1f",
                 hdr.srcId, hdr.dstId, hdr.seqNum, hdr.msgType,
                 hdr.payloadLen, (double)rssi, (double)snr);
        (void)payload;
    });

    ESP_LOGI(TAG, "Radio running.  Waiting for packets...");

    // Application main loop — add V2V logic here
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
