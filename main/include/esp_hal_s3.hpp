#pragma once

// ---------------------------------------------------------------------------
// EspHal — RadioLib Hardware Abstraction Layer for ESP32-S3
// ---------------------------------------------------------------------------
// RadioLib requires a HAL that inherits from RadioLibHal and implements all
// virtual methods for GPIO, SPI and timing.  The upstream EspHal.h supplied
// by the RadioLib repository drives SPI via raw register access (spi_dev_t /
// DPORT_* macros) which is only valid on the ESP32 classic.  This replacement
// uses the portable ESP-IDF spi_master driver so it compiles on every target
// including ESP32-S3 / S2 / C3 / C6.
// ---------------------------------------------------------------------------

#include <RadioLib.h>

// ESP-IDF peripheral + RTOS headers
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---------------------------------------------------------------------------
// Arduino-style level/direction aliases expected by RadioLibHal constructor
// ---------------------------------------------------------------------------
#define RADIOLIB_HAL_PIN_LOW    (0x0)
#define RADIOLIB_HAL_PIN_HIGH   (0x1)
#define RADIOLIB_HAL_PIN_INPUT  (0x01)
#define RADIOLIB_HAL_PIN_OUTPUT (0x03)
#define RADIOLIB_HAL_RISING     (0x01)
#define RADIOLIB_HAL_FALLING    (0x02)

class EspHal : public RadioLibHal {
public:
    // -----------------------------------------------------------------------
    // Constructor
    // Stores pin assignments; no hardware is touched until init() is called.
    // RadioLibHal constructor receives the numeric constant aliases above so
    // that RadioLib's internal code can compare against INPUT/OUTPUT/LOW/HIGH.
    // -----------------------------------------------------------------------
    EspHal(spi_host_device_t spiHost, int sck, int miso, int mosi)
        : RadioLibHal(RADIOLIB_HAL_PIN_INPUT,
                      RADIOLIB_HAL_PIN_OUTPUT,
                      RADIOLIB_HAL_PIN_LOW,
                      RADIOLIB_HAL_PIN_HIGH,
                      RADIOLIB_HAL_RISING,
                      RADIOLIB_HAL_FALLING),
          _spiHost(spiHost),
          _sck(sck),
          _miso(miso),
          _mosi(mosi),
          _spiDevice(nullptr),
          _busInitialized(false)
    {}

    // RAII destructor — free SPI resources if they were acquired
    ~EspHal() override { term(); }

    // -----------------------------------------------------------------------
    // init / term  — called by RadioLib on construction / destruction
    // -----------------------------------------------------------------------
    void init() override;
    void term() override;

    // -----------------------------------------------------------------------
    // GPIO
    // -----------------------------------------------------------------------
    void     pinMode(uint32_t pin, uint32_t mode) override;
    void     digitalWrite(uint32_t pin, uint32_t value) override;
    uint32_t digitalRead(uint32_t pin) override;

    // -----------------------------------------------------------------------
    // Interrupt
    // RadioLib calls attachInterrupt(pin, cb, mode) when setDio0Action() /
    // setDio1Action() are called.  We forward to gpio_isr_handler_add() using
    // a type cast — the callback trusts the constrained ISR signature.
    // -----------------------------------------------------------------------
    void attachInterrupt(uint32_t pin, void (*cb)(void), uint32_t mode) override;
    void detachInterrupt(uint32_t pin) override;

    // -----------------------------------------------------------------------
    // Timing
    // -----------------------------------------------------------------------
    void          delay(unsigned long ms) override;
    void          delayMicroseconds(unsigned long us) override;
    unsigned long millis() override;
    unsigned long micros() override;
    long          pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override;

    // -----------------------------------------------------------------------
    // SPI
    // RadioLib calls spiBegin/spiBeginTransaction/spiTransfer/spiEndTransaction
    // spiEnd around every SPI exchange.
    // -----------------------------------------------------------------------
    void    spiBegin() override;
    void    spiEnd() override;
    void    spiBeginTransaction() override {}   // bus locked per-transfer below
    void    spiEndTransaction()   override {}
    void    spiTransfer(uint8_t* out, size_t len, uint8_t* in) override;

    // Yield — called by RadioLib in polling loops to avoid WDT
    void yield() override { taskYIELD(); }

private:
    spi_host_device_t  _spiHost;
    int                _sck, _miso, _mosi;
    spi_device_handle_t _spiDevice;
    bool               _busInitialized;

    // Internal helpers
    void spiAddDevice();
    void spiRemoveDevice();
};
