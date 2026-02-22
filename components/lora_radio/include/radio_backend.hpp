#pragma once

/**
 * @file radio_backend.hpp
 * @brief Abstract interface for the LoRa radio hardware backend.
 *
 * @details
 * Decouples LoraRadio's state machine from the concrete SX1278 / RadioLib
 * implementation.  The production path uses Sx1278Backend; the test path
 * injects MockRadioBackend (defined in the test component) without touching
 * any real hardware.
 *
 * Return-code constants mirror RadioLib values so that @c lora_radio.cpp does
 * not need to include @c <RadioLib.h> in hot-path code.
 */

#include <cstdint>
#include <cstddef>

// ---------------------------------------------------------------------------
// Backend return-code and pin constants (mirrors RadioLib numeric values)
// ---------------------------------------------------------------------------

/// @brief Successful operation (@c RADIOLIB_ERR_NONE = 0).
static constexpr int16_t RADIO_ERR_NONE      =  0;

/// @brief CAD result: LoRa preamble detected / channel busy (@c RADIOLIB_LORA_DETECTED = 1).
static constexpr int16_t RADIO_LORA_DETECTED =  1;

/// @brief CAD result: channel clear (@c RADIOLIB_CHANNEL_FREE = 0).
static constexpr int16_t RADIO_CHANNEL_FREE  =  0;

/// @brief Rising-edge interrupt direction (@c RADIOLIB_HAL_RISING = 0x01).
static constexpr uint32_t RADIO_RISING       = 0x01u;

/// @brief "Pin not connected" sentinel (@c RADIOLIB_NC = -1).
static constexpr int RADIO_PIN_NC            = -1;

// ---------------------------------------------------------------------------
// IRadioBackend — pure-virtual radio hardware interface
// ---------------------------------------------------------------------------

/**
 * @brief Pure-virtual interface that abstracts all radio hardware operations.
 *
 * @details
 * Every method maps 1-to-1 onto a RadioLib SX1278 API call.  Concrete
 * implementations include Sx1278Backend (production) and MockRadioBackend
 * (unit tests).  LoraRadio's state machine holds an @c IRadioBackend* and
 * never calls RadioLib or touches SPI directly.
 */
class IRadioBackend {
public:
    virtual ~IRadioBackend() = default;

    // -----------------------------------------------------------------------
    // Initialisation / configuration (called once during LoraRadio::init)
    // -----------------------------------------------------------------------

    /// @brief Configure the radio modem with the given RF parameters.
    /// Equivalent to SX1278::begin().
    /// @param freq     Centre frequency in MHz.
    /// @param bw       Signal bandwidth in kHz.
    /// @param sf       Spreading factor (6–12).
    /// @param cr       Coding-rate denominator (5–8, meaning 4/5 … 4/8).
    /// @param syncWord 1-byte sync word (0x12 = private, 0x34 = LoRaWAN).
    /// @param power    TX power in dBm.
    /// @return RADIO_ERR_NONE on success, negative error code on failure.
    virtual int16_t begin(float   freq,
                          float   bw,
                          uint8_t sf,
                          uint8_t cr,
                          uint8_t syncWord,
                          int8_t  power) = 0;

    /// @brief Attach the DIO0 interrupt callback.
    /// @param cb   Function to call when DIO0 fires.
    /// @param dir  Trigger direction (e.g. @c RADIO_RISING).
    virtual void setDio0Action(void (*cb)(void), uint32_t dir) = 0;

    /// @brief Detach the DIO0 interrupt callback.
    virtual void clearDio0Action() = 0;

    /// @brief Attach the DIO1 interrupt callback.
    /// @param cb   Function to call when DIO1 fires.
    /// @param dir  Trigger direction (e.g. @c RADIO_RISING).
    virtual void setDio1Action(void (*cb)(void), uint32_t dir) = 0;

    /// @brief Detach the DIO1 interrupt callback.
    virtual void clearDio1Action() = 0;

    // -----------------------------------------------------------------------
    // Channel Activity Detection
    // -----------------------------------------------------------------------

    /// @brief Begin a CAD scan.
    /// DIO0 fires when the scan finishes (channel clear); DIO1 fires if busy.
    /// @return RADIO_ERR_NONE on success, negative error code on failure.
    virtual int16_t startChannelScan() = 0;

    /// @brief Read the CAD result after DIO0 fires.
    /// @return @c RADIO_LORA_DETECTED (channel busy) or @c RADIO_CHANNEL_FREE.
    virtual int16_t getChannelScanResult() = 0;

    // -----------------------------------------------------------------------
    // Transmit
    // -----------------------------------------------------------------------

    /// @brief Begin an asynchronous transmission.
    /// DIO0 fires when TX is complete.
    /// @param data  Pointer to the frame bytes to transmit.
    /// @param len   Number of bytes to transmit.
    /// @return RADIO_ERR_NONE on success, negative error code on failure.
    virtual int16_t startTransmit(uint8_t* data, size_t len) = 0;

    /// @brief Finalise a completed transmission.
    /// Must be called after DIO0 fires for TX-done to reset modem state.
    /// @return RADIO_ERR_NONE on success, negative error code on failure.
    virtual int16_t finishTransmit() = 0;

    // -----------------------------------------------------------------------
    // Receive
    // -----------------------------------------------------------------------

    /// @brief Enter continuous receive mode.
    /// DIO0 fires when a packet is ready to read.
    /// @return RADIO_ERR_NONE on success, negative error code on failure.
    virtual int16_t startReceive() = 0;

    /// @brief Return the byte-length of the most recently received packet.
    virtual size_t  getPacketLength() = 0;

    /// @brief Copy the received packet bytes into @p data.
    /// @param data  Output buffer (must be at least @p len bytes).
    /// @param len   Number of bytes to read.
    /// @return RADIO_ERR_NONE on success, negative error code on CRC failure etc.
    virtual int16_t readData(uint8_t* data, size_t len) = 0;

    /// @brief RSSI of the most recently received packet (dBm).
    virtual float getRSSI() = 0;

    /// @brief SNR of the most recently received packet (dB).
    virtual float getSNR() = 0;

    // -----------------------------------------------------------------------
    // Power management
    // -----------------------------------------------------------------------

    /// @brief Enter low-power modem sleep.
    /// @return RADIO_ERR_NONE on success, negative error code on failure.
    virtual int16_t sleep() = 0;

    /// @brief Return to standby mode (cancels sleep or continuous receive).
    /// @return RADIO_ERR_NONE on success, negative error code on failure.
    virtual int16_t standby() = 0;
};
