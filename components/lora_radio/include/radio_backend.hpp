#pragma once

// ---------------------------------------------------------------------------
// radio_backend.hpp — Abstract interface for the LoRa radio hardware
// ---------------------------------------------------------------------------
// Decouples LoraRadio's state machine from the concrete SX1278 / RadioLib
// implementation.  The production path uses Sx1278Backend; the test path
// injects MockRadioBackend (defined in the test component) without touching
// any real hardware.
//
// Return-code constants mirror RadioLib values so that lora_radio.cpp does
// not need to include <RadioLib.h> in hot-path code.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <cstddef>

// ---------------------------------------------------------------------------
// Backend return-code and pin constants (mirrors RadioLib numeric values)
// ---------------------------------------------------------------------------

/// Successful operation (RADIOLIB_ERR_NONE = 0)
static constexpr int16_t RADIO_ERR_NONE      =  0;

/// CAD result: LoRa preamble detected / channel busy (RADIOLIB_LORA_DETECTED = 1)
static constexpr int16_t RADIO_LORA_DETECTED =  1;

/// CAD result: channel clear (RADIOLIB_CHANNEL_FREE = 0)
static constexpr int16_t RADIO_CHANNEL_FREE  =  0;

/// Rising-edge interrupt direction (RADIOLIB_HAL_RISING = 0x01)
static constexpr uint32_t RADIO_RISING       = 0x01u;

/// "Pin not connected" sentinel (RADIOLIB_NC = -1)
static constexpr int RADIO_PIN_NC            = -1;

// ---------------------------------------------------------------------------
// IRadioBackend — pure-virtual radio hardware interface
// ---------------------------------------------------------------------------

class IRadioBackend {
public:
    virtual ~IRadioBackend() = default;

    // -----------------------------------------------------------------------
    // Initialisation / configuration (called once during LoraRadio::init)
    // -----------------------------------------------------------------------

    /// Configure the radio modem with the given RF parameters.
    /// Equivalent to SX1278::begin().
    /// @return RADIO_ERR_NONE on success, negative error code on failure.
    virtual int16_t begin(float   freq,
                          float   bw,
                          uint8_t sf,
                          uint8_t cr,
                          uint8_t syncWord,
                          int8_t  power) = 0;

    /// Attach the DIO0 interrupt callback.
    virtual void setDio0Action(void (*cb)(void), uint32_t dir) = 0;

    /// Detach the DIO0 interrupt callback.
    virtual void clearDio0Action() = 0;

    /// Attach the DIO1 interrupt callback.
    virtual void setDio1Action(void (*cb)(void), uint32_t dir) = 0;

    /// Detach the DIO1 interrupt callback.
    virtual void clearDio1Action() = 0;

    // -----------------------------------------------------------------------
    // Channel Activity Detection
    // -----------------------------------------------------------------------

    /// Begin a CAD scan.  DIO0 fires when complete, DIO1 fires if busy.
    virtual int16_t startChannelScan() = 0;

    /// Read the CAD result after DIO0 fires.
    /// @return RADIO_LORA_DETECTED or RADIO_CHANNEL_FREE.
    virtual int16_t getChannelScanResult() = 0;

    // -----------------------------------------------------------------------
    // Transmit
    // -----------------------------------------------------------------------

    /// Begin an asynchronous transmission.  DIO0 fires when TX is complete.
    virtual int16_t startTransmit(uint8_t* data, size_t len) = 0;

    /// Must be called after DIO0 fires for TX done to reset modem state.
    virtual int16_t finishTransmit() = 0;

    // -----------------------------------------------------------------------
    // Receive
    // -----------------------------------------------------------------------

    /// Enter continuous receive mode.  DIO0 fires when a packet is ready.
    virtual int16_t startReceive() = 0;

    /// Return the number of bytes in the most recently received packet.
    virtual size_t  getPacketLength() = 0;

    /// Copy the received packet bytes into @p data.
    /// @return RADIO_ERR_NONE on success, negative error code on CRC fail etc.
    virtual int16_t readData(uint8_t* data, size_t len) = 0;

    /// RSSI of the most recently received packet (dBm).
    virtual float getRSSI() = 0;

    /// SNR of the most recently received packet (dB).
    virtual float getSNR() = 0;

    // -----------------------------------------------------------------------
    // Power management
    // -----------------------------------------------------------------------

    /// Enter low-power modem sleep.
    virtual int16_t sleep() = 0;

    /// Return to standby (cancel sleep/receive).
    virtual int16_t standby() = 0;
};
