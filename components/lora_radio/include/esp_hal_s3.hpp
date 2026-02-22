#pragma once

/**
 * @file esp_hal_s3.hpp
 * @brief RadioLib Hardware Abstraction Layer for ESP32-S3 using the ESP-IDF SPI master driver.
 *
 * @details
 * RadioLib requires a HAL that inherits from @c RadioLibHal and implements all
 * virtual methods for GPIO, SPI and timing.  The upstream @c EspHal.h supplied
 * by the RadioLib repository drives SPI via raw register access (@c spi_dev_t /
 * @c DPORT_* macros) which is only valid on the ESP32 classic.  This replacement
 * uses the portable ESP-IDF @c spi_master driver so it compiles on every target
 * including ESP32-S3 / S2 / C3 / C6.
 */

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

/**
 * @brief RadioLib HAL implementation using the ESP-IDF @c spi_master driver.
 *
 * Inherits from @c RadioLibHal and provides concrete implementations of GPIO,
 * SPI and timing primitives needed by RadioLib.  Uses the portable
 * @c spi_bus_initialize / @c spi_device_polling_transmit API instead of bare
 * metal register access, making it compatible with all ESP32 variants.
 */
class EspHal : public RadioLibHal {
public:
    // -----------------------------------------------------------------------
    // Constructor
    // Stores pin assignments; no hardware is touched until init() is called.
    // RadioLibHal constructor receives the numeric constant aliases above so
    // that RadioLib's internal code can compare against INPUT/OUTPUT/LOW/HIGH.
    // -----------------------------------------------------------------------

    /**
     * @brief Construct an EspHal instance.
     *
     * No hardware is initialised here; call init() to bring up the SPI bus.
     *
     * @param spiHost  ESP-IDF SPI host (e.g. @c SPI2_HOST).
     * @param sck      GPIO number for the SPI clock line.
     * @param miso     GPIO number for the MISO line.
     * @param mosi     GPIO number for the MOSI line.
     */
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

    /// @brief RAII destructor — frees the SPI bus and device if they were acquired.
    ~EspHal() override { term(); }

    // -----------------------------------------------------------------------
    // init / term  — called by RadioLib on construction / destruction
    // -----------------------------------------------------------------------

    /// @brief Initialise the SPI bus and add the radio device.
    /// Called automatically by RadioLib when the @c Module is constructed.
    void init() override;

    /// @brief Release the SPI device and free the SPI bus.
    /// Safe to call multiple times; subsequent calls are no-ops.
    void term() override;

    // -----------------------------------------------------------------------
    // GPIO
    // -----------------------------------------------------------------------

    /// @brief Configure a GPIO pin direction.
    /// @param pin   GPIO number.
    /// @param mode  @c RADIOLIB_HAL_PIN_INPUT or @c RADIOLIB_HAL_PIN_OUTPUT.
    void     pinMode(uint32_t pin, uint32_t mode) override;

    /// @brief Drive a GPIO output pin.
    /// @param pin    GPIO number.
    /// @param value  @c RADIOLIB_HAL_PIN_HIGH or @c RADIOLIB_HAL_PIN_LOW.
    void     digitalWrite(uint32_t pin, uint32_t value) override;

    /// @brief Read a GPIO input pin.
    /// @param pin  GPIO number.
    /// @return 1 if the pin is high, 0 if low.
    uint32_t digitalRead(uint32_t pin) override;

    // -----------------------------------------------------------------------
    // Interrupt
    // RadioLib calls attachInterrupt(pin, cb, mode) when setDio0Action() /
    // setDio1Action() are called.  We forward to gpio_isr_handler_add() using
    // a type cast — the callback trusts the constrained ISR signature.
    // -----------------------------------------------------------------------

    /// @brief Attach an interrupt handler to a GPIO pin.
    /// @param pin   GPIO number.
    /// @param cb    Callback to invoke on the configured edge.
    /// @param mode  @c RADIOLIB_HAL_RISING, @c RADIOLIB_HAL_FALLING, or any edge.
    void attachInterrupt(uint32_t pin, void (*cb)(void), uint32_t mode) override;

    /// @brief Detach the interrupt handler from a GPIO pin.
    /// @param pin  GPIO number.
    void detachInterrupt(uint32_t pin) override;

    // -----------------------------------------------------------------------
    // Timing
    // -----------------------------------------------------------------------

    /// @brief Block for at least @p ms milliseconds (calls @c vTaskDelay).
    void          delay(unsigned long ms) override;

    /// @brief Busy-wait for at least @p us microseconds.
    void          delayMicroseconds(unsigned long us) override;

    /// @brief Return elapsed time in milliseconds since boot.
    unsigned long millis() override;

    /// @brief Return elapsed time in microseconds since boot.
    unsigned long micros() override;

    /// @brief Measure the duration of a pulse on @p pin.
    /// @param pin      GPIO number to measure.
    /// @param state    Logic level to measure (0 = low, 1 = high).
    /// @param timeout  Maximum wait time in microseconds.
    /// @return Pulse width in microseconds, or 0 on timeout.
    long          pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override;

    // -----------------------------------------------------------------------
    // SPI
    // RadioLib calls spiBegin/spiBeginTransaction/spiTransfer/spiEndTransaction
    // spiEnd around every SPI exchange.
    // -----------------------------------------------------------------------

    /// @brief Initialise the SPI bus (idempotent).
    void    spiBegin() override;

    /// @brief Release the SPI bus (delegates to term()).
    void    spiEnd() override;

    /// @brief No-op: bus is locked per-transfer inside spiTransfer().
    void    spiBeginTransaction() override {}

    /// @brief No-op: bus is released per-transfer inside spiTransfer().
    void    spiEndTransaction()   override {}

    /// @brief Perform a full-duplex SPI transfer.
    /// @param out  Bytes to transmit (must be @p len bytes long).
    /// @param len  Number of bytes to exchange.
    /// @param in   Buffer to receive bytes into (must be @p len bytes long).
    void    spiTransfer(uint8_t* out, size_t len, uint8_t* in) override;

    /// @brief Yield to the FreeRTOS scheduler (called by RadioLib in polling loops).
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
