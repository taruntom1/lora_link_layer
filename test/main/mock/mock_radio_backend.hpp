#pragma once

// ---------------------------------------------------------------------------
// mock_radio_backend.hpp — Test double for IRadioBackend
// ---------------------------------------------------------------------------
// Implements the full IRadioBackend interface without touching any hardware.
// Designed for on-target FreeRTOS unit tests running in the same binary as
// the LoraRadio component.
//
// Usage pattern:
//     MockRadioBackend mock;
//     LoraRadio radio;
//     radio.initForTest(&mock);
//     // ... call send(), trigger DIO events, assert mock counters ...
//     radio.deinit();
//
// Triggering simulated hardware events:
//     mock.fireDio0();   // simulates DIO0 pulse (TX done / RX done / CAD clear)
//     mock.fireDio1();   // simulates DIO1 pulse (CAD busy)
//
// The fire* methods call the stored ISR trampolines which call
// xTaskNotifyFromISR.  Calling xTaskNotifyFromISR from a task context is
// valid in ESP-IDF FreeRTOS (task context is treated as a low-priority ISR
// context for notification purposes).
// ---------------------------------------------------------------------------

#include "radio_backend.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

class MockRadioBackend final : public IRadioBackend {
public:
    // -----------------------------------------------------------------------
    // Configurable return values
    // Tests set these before calling the code under test.
    // -----------------------------------------------------------------------
    int16_t beginResult          = RADIO_ERR_NONE;
    int16_t startChannelScanResult = RADIO_ERR_NONE;
    int16_t channelScanResult    = RADIO_CHANNEL_FREE;  // RADIO_LORA_DETECTED to simulate busy
    int16_t startTransmitResult  = RADIO_ERR_NONE;
    int16_t finishTransmitResult = RADIO_ERR_NONE;
    int16_t startReceiveResult   = RADIO_ERR_NONE;
    int16_t readDataResult       = RADIO_ERR_NONE;

    // -----------------------------------------------------------------------
    // Injected RX packet (delivered when the RX callback fires)
    // -----------------------------------------------------------------------
    uint8_t  rxBuf[255]  = {};
    size_t   rxLen       = 0;
    float    rxRssi      = -80.0f;
    float    rxSnr       =   7.0f;

    // -----------------------------------------------------------------------
    // Call counters
    // -----------------------------------------------------------------------
    int beginCount           = 0;
    int setDio0ActionCount   = 0;
    int setDio1ActionCount   = 0;
    int clearDio0ActionCount = 0;
    int clearDio1ActionCount = 0;
    int startChannelScanCount  = 0;
    int getChannelScanResultCount = 0;
    int startTransmitCount   = 0;
    int finishTransmitCount  = 0;
    int startReceiveCount    = 0;
    int getPacketLengthCount = 0;
    int readDataCount        = 0;
    int sleepCount           = 0;
    int standbyCount         = 0;

    // Last startTransmit arguments — inspect to verify frame content
    uint8_t  lastTxBuf[255] = {};
    size_t   lastTxLen      = 0;

    // -----------------------------------------------------------------------
    // Reset all counters and captured state (call between tests)
    // -----------------------------------------------------------------------
    void reset()
    {
        beginResult           = RADIO_ERR_NONE;
        startChannelScanResult = RADIO_ERR_NONE;
        channelScanResult     = RADIO_CHANNEL_FREE;
        startTransmitResult   = RADIO_ERR_NONE;
        finishTransmitResult  = RADIO_ERR_NONE;
        startReceiveResult    = RADIO_ERR_NONE;
        readDataResult        = RADIO_ERR_NONE;
        rxLen                 = 0;
        rxRssi                = -80.0f;
        rxSnr                 =   7.0f;

        beginCount = setDio0ActionCount = setDio1ActionCount = 0;
        clearDio0ActionCount = clearDio1ActionCount = 0;
        startChannelScanCount = getChannelScanResultCount = 0;
        startTransmitCount = finishTransmitCount = 0;
        startReceiveCount = getPacketLengthCount = readDataCount = 0;
        sleepCount = standbyCount = 0;

        std::memset(lastTxBuf, 0, sizeof(lastTxBuf));
        lastTxLen = 0;
        _dio0Cb   = nullptr;
        _dio1Cb   = nullptr;
    }

    // -----------------------------------------------------------------------
    // Simulate hardware interrupt events
    // -----------------------------------------------------------------------

    /// Simulate a DIO0 pulse (TX done / RX done / CAD no-activity).
    void fireDio0()
    {
        if (_dio0Cb) _dio0Cb();
    }

    /// Simulate a DIO1 pulse (CAD preamble / channel busy).
    void fireDio1()
    {
        if (_dio1Cb) _dio1Cb();
    }

    // -----------------------------------------------------------------------
    // IRadioBackend implementation
    // -----------------------------------------------------------------------

    int16_t begin(float, float, uint8_t, uint8_t, uint8_t, int8_t) override
    {
        ++beginCount;
        return beginResult;
    }

    void setDio0Action(void (*cb)(void), uint32_t) override
    {
        ++setDio0ActionCount;
        _dio0Cb = cb;
    }

    void clearDio0Action() override
    {
        ++clearDio0ActionCount;
        _dio0Cb = nullptr;
    }

    void setDio1Action(void (*cb)(void), uint32_t) override
    {
        ++setDio1ActionCount;
        _dio1Cb = cb;
    }

    void clearDio1Action() override
    {
        ++clearDio1ActionCount;
        _dio1Cb = nullptr;
    }

    int16_t startChannelScan() override
    {
        ++startChannelScanCount;
        return startChannelScanResult;
    }

    int16_t getChannelScanResult() override
    {
        ++getChannelScanResultCount;
        return channelScanResult;
    }

    int16_t startTransmit(uint8_t* data, size_t len) override
    {
        ++startTransmitCount;
        size_t copy = len < sizeof(lastTxBuf) ? len : sizeof(lastTxBuf);
        std::memcpy(lastTxBuf, data, copy);
        lastTxLen = copy;
        return startTransmitResult;
    }

    int16_t finishTransmit() override
    {
        ++finishTransmitCount;
        return finishTransmitResult;
    }

    int16_t startReceive() override
    {
        ++startReceiveCount;
        return startReceiveResult;
    }

    size_t getPacketLength() override
    {
        ++getPacketLengthCount;
        return rxLen;
    }

    int16_t readData(uint8_t* data, size_t len) override
    {
        ++readDataCount;
        size_t copy = rxLen < len ? rxLen : len;
        std::memcpy(data, rxBuf, copy);
        return readDataResult;
    }

    float getRSSI() override { return rxRssi; }
    float getSNR()  override { return rxSnr;  }

    int16_t sleep()   override { ++sleepCount;   return RADIO_ERR_NONE; }
    int16_t standby() override { ++standbyCount; return RADIO_ERR_NONE; }

private:
    void (*_dio0Cb)(void) = nullptr;
    void (*_dio1Cb)(void) = nullptr;
};
