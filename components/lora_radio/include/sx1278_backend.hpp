#pragma once

// ---------------------------------------------------------------------------
// sx1278_backend.hpp — Concrete IRadioBackend wrapping a RadioLib SX1278
// ---------------------------------------------------------------------------
// Thin adapter: every virtual call is forwarded to the real SX1278 object.
// This keeps RadioLib headers out of the state-machine translation unit
// (lora_radio.cpp) while still reaching the real hardware at runtime.
// ---------------------------------------------------------------------------

#include "radio_backend.hpp"
#include <RadioLib.h>   // SX1278, PhysicalLayer

class Sx1278Backend final : public IRadioBackend {
public:
    explicit Sx1278Backend(SX1278* radio) : _radio(radio) {}
    ~Sx1278Backend() override = default;

    // ------------------------------------------------------------------
    // IRadioBackend implementation — all calls forwarded to SX1278
    // ------------------------------------------------------------------

    int16_t begin(float freq, float bw, uint8_t sf, uint8_t cr,
                  uint8_t syncWord, int8_t power) override
    {
        return _radio->begin(freq, bw, sf, cr, syncWord, power);
    }

    void setDio0Action(void (*cb)(void), uint32_t dir) override
    {
        _radio->setDio0Action(cb, dir);
    }

    void clearDio0Action() override
    {
        _radio->clearDio0Action();
    }

    void setDio1Action(void (*cb)(void), uint32_t dir) override
    {
        _radio->setDio1Action(cb, dir);
    }

    void clearDio1Action() override
    {
        _radio->clearDio1Action();
    }

    int16_t startChannelScan() override
    {
        return _radio->startChannelScan();
    }

    int16_t getChannelScanResult() override
    {
        return _radio->getChannelScanResult();
    }

    int16_t startTransmit(uint8_t* data, size_t len) override
    {
        return _radio->startTransmit(data, len);
    }

    int16_t finishTransmit() override
    {
        return _radio->finishTransmit();
    }

    int16_t startReceive() override
    {
        return _radio->startReceive();
    }

    size_t getPacketLength() override
    {
        return _radio->getPacketLength();
    }

    int16_t readData(uint8_t* data, size_t len) override
    {
        return _radio->readData(data, len);
    }

    float getRSSI() override { return _radio->getRSSI(); }
    float getSNR()  override { return _radio->getSNR();  }

    int16_t sleep()   override { return _radio->sleep();   }
    int16_t standby() override { return _radio->standby(); }

private:
    SX1278* _radio;   ///< Owned externally (by LoraRadio::init)
};
