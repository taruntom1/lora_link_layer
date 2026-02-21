// ---------------------------------------------------------------------------
// esp_hal_s3.cpp — RadioLib HAL implementation via ESP-IDF spi_master driver
// ---------------------------------------------------------------------------
// Replaces the upstream EspHal.h which uses bare-metal SPI register access
// (spi_dev_t / DPORT_*) that only compiles on ESP32 classic.  This file uses
// the portable spi_bus_initialize / spi_device_polling_transmit API and works
// on ESP32-S3 (and every other ESP32 variant).
// ---------------------------------------------------------------------------

#include "esp_hal_s3.hpp"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "EspHal";

// ===========================================================================
// Lifecycle
// ===========================================================================

void EspHal::init()
{
    spiBegin();
}

void EspHal::term()
{
    if (_busInitialized) {
        spiRemoveDevice();
        spi_bus_free(_spiHost);
        _busInitialized = false;
        ESP_LOGD(TAG, "SPI bus freed (host %d)", (int)_spiHost);
    }
}

// ===========================================================================
// GPIO
// ===========================================================================

void EspHal::pinMode(uint32_t pin, uint32_t mode)
{
    if (pin == RADIOLIB_NC) return;

    gpio_config_t cfg = {};
    cfg.pin_bit_mask = (1ULL << pin);
    cfg.intr_type    = GPIO_INTR_DISABLE;
    cfg.pull_up_en   = GPIO_PULLUP_DISABLE;
    cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;

    if (mode == RADIOLIB_HAL_PIN_OUTPUT) {
        cfg.mode = GPIO_MODE_OUTPUT;
    } else {
        // INPUT or INPUT_PULLUP — treat everything else as input
        cfg.mode = GPIO_MODE_INPUT;
    }

    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "gpio_config pin=%u failed: %s", pin, esp_err_to_name(err));
    }
}

void EspHal::digitalWrite(uint32_t pin, uint32_t value)
{
    if (pin == RADIOLIB_NC) return;
    gpio_set_level((gpio_num_t)pin, value ? 1 : 0);
}

uint32_t EspHal::digitalRead(uint32_t pin)
{
    if (pin == RADIOLIB_NC) return 0;
    return (uint32_t)gpio_get_level((gpio_num_t)pin);
}

// ===========================================================================
// Interrupts
// ===========================================================================
// RadioLib calls attachInterrupt(pin, cb, mode) when setDio0Action() /
// setDio1Action() are invoked.  The supplied `cb` is later stored in the
// LoraRadio class as static void (*)(void) trampolines; we register them
// directly with the GPIO ISR service using a type-cast.
// The ISR service is installed once; repeated calls are harmless.
// ===========================================================================

void EspHal::attachInterrupt(uint32_t pin, void (*cb)(void), uint32_t mode)
{
    if (pin == RADIOLIB_NC) return;

    // Install the shared GPIO ISR service if not already done.
    // ESP_INTR_FLAG_IRAM keeps ISR code executable during flash operations.
    gpio_install_isr_service((int)ESP_INTR_FLAG_IRAM);

    gpio_int_type_t intType = GPIO_INTR_DISABLE;
    if (mode == RADIOLIB_HAL_RISING) {
        intType = GPIO_INTR_POSEDGE;
    } else if (mode == RADIOLIB_HAL_FALLING) {
        intType = GPIO_INTR_NEGEDGE;
    } else {
        intType = GPIO_INTR_ANYEDGE;
    }

    gpio_set_intr_type((gpio_num_t)pin, intType);

    // Cast void(*)(void) to the ISR handler signature void(*)(void*).
    // The NULL context argument is ignored by our trampolines in LoraRadio.
    gpio_isr_handler_add((gpio_num_t)pin,
                         reinterpret_cast<gpio_isr_t>(cb),
                         nullptr);

    ESP_LOGD(TAG, "ISR attached: pin=%u mode=%u", pin, mode);
}

void EspHal::detachInterrupt(uint32_t pin)
{
    if (pin == RADIOLIB_NC) return;
    gpio_isr_handler_remove((gpio_num_t)pin);
    gpio_set_intr_type((gpio_num_t)pin, GPIO_INTR_DISABLE);
    ESP_LOGD(TAG, "ISR detached: pin=%u", pin);
}

// ===========================================================================
// Timing
// ===========================================================================

void EspHal::delay(unsigned long ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

void EspHal::delayMicroseconds(unsigned long us)
{
    // esp_timer_get_time() returns microseconds since boot — busy-wait.
    if (us == 0) return;
    uint64_t start = (uint64_t)esp_timer_get_time();
    uint64_t end   = start + us;
    // Handle 64-bit overflow (extremely unlikely but correct)
    if (end < start) {
        while ((uint64_t)esp_timer_get_time() > end) { /* spin */ }
    }
    while ((uint64_t)esp_timer_get_time() < end) { /* spin */ }
}

unsigned long EspHal::millis()
{
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

unsigned long EspHal::micros()
{
    return (unsigned long)esp_timer_get_time();
}

long EspHal::pulseIn(uint32_t pin, uint32_t state, unsigned long timeout)
{
    if (pin == RADIOLIB_NC) return 0;
    this->pinMode(pin, RADIOLIB_HAL_PIN_INPUT);

    uint32_t start   = this->micros();
    uint32_t curtick = start;

    // Wait until pin leaves the initial state
    while ((uint32_t)this->digitalRead(pin) == state) {
        if ((this->micros() - curtick) > timeout) return 0;
    }
    // Measure how long pin is in `state`
    while ((uint32_t)this->digitalRead(pin) != state) {
        if ((this->micros() - curtick) > timeout) return 0;
    }
    uint32_t pulseStart = this->micros();
    while ((uint32_t)this->digitalRead(pin) == state) {
        if ((this->micros() - curtick) > timeout) return 0;
    }
    return (long)(this->micros() - pulseStart);
}

// ===========================================================================
// SPI
// ===========================================================================

void EspHal::spiBegin()
{
    if (_busInitialized) return;

    // --- Configure the SPI bus ---
    spi_bus_config_t busCfg = {};
    busCfg.mosi_io_num     = _mosi;
    busCfg.miso_io_num     = _miso;
    busCfg.sclk_io_num     = _sck;
    busCfg.quadwp_io_num   = -1;
    busCfg.quadhd_io_num   = -1;
    busCfg.max_transfer_sz = 256;   // SX1278 max frame = 255 bytes + 1 addr

    esp_err_t err = spi_bus_initialize((spi_host_device_t)_spiHost,
                                       &busCfg,
                                       SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        // ESP_ERR_INVALID_STATE means the bus is already initialised by
        // another driver — that is acceptable, we just add our device.
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(err));
        return;
    }

    _busInitialized = true;
    spiAddDevice();
    ESP_LOGD(TAG, "SPI bus initialised (host %d)", (int)_spiHost);
}

void EspHal::spiEnd()
{
    term();
}

void EspHal::spiAddDevice()
{
    // NSS is controlled by RadioLib via digitalWrite(), so spics_io_num = -1.
    // SX1278 uses SPI mode 0 (CPOL=0, CPHA=0), MSB first.
    // 8 MHz is well within SX1278's 10 MHz SPI limit and safe through GPIO
    // matrix on ESP32-S3.
    spi_device_interface_config_t devCfg = {};
    devCfg.command_bits     = 0;
    devCfg.address_bits     = 0;
    devCfg.dummy_bits       = 0;
    devCfg.mode             = 0;            // SPI mode 0
    devCfg.clock_speed_hz   = 8 * 1000 * 1000;  // 8 MHz
    devCfg.spics_io_num     = -1;           // software CS via digitalWrite
    devCfg.queue_size       = 1;
    devCfg.flags            = 0;

    esp_err_t err = spi_bus_add_device((spi_host_device_t)_spiHost,
                                       &devCfg,
                                       &_spiDevice);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device failed: %s", esp_err_to_name(err));
        _spiDevice = nullptr;
    }
}

void EspHal::spiRemoveDevice()
{
    if (_spiDevice) {
        spi_bus_remove_device(_spiDevice);
        _spiDevice = nullptr;
    }
}

// ---------------------------------------------------------------------------
// spiTransfer — full-duplex byte-by-byte exchange with the SX1278
// ---------------------------------------------------------------------------
// RadioLib calls this for every SPI frame (register read/write).
// We use polling transactions rather than interrupt transactions to avoid
// the ~26 µs interrupt overhead for short 1–3 byte SPI frames typical of
// SX1278 register access.  The radio task is the sole SPI user, so there is
// no bus contention.
// ---------------------------------------------------------------------------
void EspHal::spiTransfer(uint8_t* out, size_t len, uint8_t* in)
{
    if (!_spiDevice || len == 0) return;

    // Acquire the bus once for the whole multi-byte transfer to prevent
    // any other driver interleaving between bytes of the same SX1278 frame.
    spi_device_acquire_bus(_spiDevice, portMAX_DELAY);

    for (size_t i = 0; i < len; i++) {
        spi_transaction_t t = {};   // stack-allocated — no heap per byte
        t.length    = 8;            // 8 bits per byte
        t.rxlength  = 8;
        t.flags     = SPI_TRANS_USE_TXDATA | SPI_TRANS_USE_RXDATA;
        t.tx_data[0] = out[i];

        esp_err_t err = spi_device_polling_transmit(_spiDevice, &t);
        if (err == ESP_OK) {
            in[i] = t.rx_data[0];
        } else {
            ESP_LOGW(TAG, "SPI transfer byte %zu failed: %s",
                     i, esp_err_to_name(err));
            in[i] = 0xFF;
        }
    }

    spi_device_release_bus(_spiDevice);
}
