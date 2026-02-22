// ---------------------------------------------------------------------------
// receiver/main/main.cpp — LoRa Receiver Example
// ---------------------------------------------------------------------------
// Demonstrates how to use the lora_radio component to receive and log LoRa
// packets.  Configure GPIO pins via menuconfig before flashing
// (Component config → LoRa Radio).
// ---------------------------------------------------------------------------

#include <cstring>
#include "lora_radio.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "lora_receiver";

// ---------------------------------------------------------------------------
// RX callback — invoked from the radio task for every received packet.
// Keep the implementation short; avoid blocking calls inside the callback.
// ---------------------------------------------------------------------------
static void on_receive(const PacketHeader &hdr,
                       const uint8_t      *payload,
                       float               rssi,
                       float               snr)
{
    // Null-terminate the payload so it can be printed as a C string.
    char buf[LORA_MAX_PAYLOAD + 1];
    size_t len = (hdr.payloadLen < LORA_MAX_PAYLOAD) ? hdr.payloadLen
                                                      : LORA_MAX_PAYLOAD;
    memcpy(buf, payload, len);
    buf[len] = '\0';

    ESP_LOGI(TAG,
             "RX  src=0x%04X dst=0x%04X seq=%u type=0x%02X "
             "RSSI=%.1f dBm SNR=%.1f dB  payload=\"%s\"",
             hdr.srcId, hdr.dstId, hdr.seqNum, hdr.msgType,
             (double)rssi, (double)snr, buf);
}

// ---------------------------------------------------------------------------
// Application entry point
// ---------------------------------------------------------------------------
static LoraRadio radio;  // Config defaults from Kconfig / sdkconfig.defaults

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "LoRa receiver example starting (node 0x%04X)",
             (unsigned)CONFIG_LORA_NODE_ID);

    esp_err_t err = radio.init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "radio.init() failed: %s", esp_err_to_name(err));
        return;
    }

    radio.setRxCallback(on_receive);

    ESP_LOGI(TAG, "Radio initialised. Waiting for packets...");

    // Nothing else to do — the radio task calls on_receive() automatically.
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
