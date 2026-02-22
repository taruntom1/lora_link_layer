#pragma once

/**
 * @file sx1278_backend.hpp
 * @brief Concrete IRadioBackend implementation wrapping a RadioLib SX1278.
 *
 * @details
 * Thin adapter: every virtual call is forwarded to the real @c SX1278 object.
 * This keeps RadioLib headers out of the state-machine translation unit
 * (@c lora_radio.cpp) while still reaching the real hardware at runtime.
 */

#include "radio_backend.hpp"
#include <RadioLib.h>   // SX1278, PhysicalLayer

/**
 * @brief IRadioBackend adapter for the RadioLib SX1278 driver.
 *
 * Forwards every IRadioBackend virtual call directly to the underlying
 * @c SX1278 instance.  The @c SX1278 object is created and owned by
 * LoraRadio::init(); this class only holds a non-owning pointer.
 */
class Sx1278Backend final : public IRadioBackend {
public:
    /// @brief Construct the backend around an already-created SX1278 object.
    /// @param radio  Non-owning pointer to the SX1278 instance.  Must remain
    ///               valid for the lifetime of this Sx1278Backend.
    explicit Sx1278Backend(SX1278* radio) : _radio(radio) {}
    ~Sx1278Backend() override = default;

    // ------------------------------------------------------------------
    // IRadioBackend implementation — all calls forwarded to SX1278
    // ------------------------------------------------------------------

    /// @brief Forwarded to SX1278::begin().
    int16_t begin(float freq, float bw, uint8_t sf, uint8_t cr,
                  uint8_t syncWord, int8_t power) override
    {
        return _radio->begin(freq, bw, sf, cr, syncWord, power);
    }

    /// @brief Forwarded to SX1278::setDio0Action().
    void setDio0Action(void (*cb)(void), uint32_t dir) override
    {
        _radio->setDio0Action(cb, dir);
    }

    /// @brief Forwarded to SX1278::clearDio0Action().
    void clearDio0Action() override
    {
        _radio->clearDio0Action();
    }

    /// @brief Forwarded to SX1278::setDio1Action().
    void setDio1Action(void (*cb)(void), uint32_t dir) override
    {
        _radio->setDio1Action(cb, dir);
    }

    /// @brief Forwarded to SX1278::clearDio1Action().
    void clearDio1Action() override
    {
        _radio->clearDio1Action();
    }

    /// @brief Forwarded to SX1278::startChannelScan().
    int16_t startChannelScan() override
    {
        return _radio->startChannelScan();
    }

    /// @brief Forwarded to SX1278::getChannelScanResult().
    int16_t getChannelScanResult() override
    {
        return _radio->getChannelScanResult();
    }

    /// @brief Forwarded to SX1278::startTransmit().
    int16_t startTransmit(uint8_t* data, size_t len) override
    {
        return _radio->startTransmit(data, len);
    }

    /// @brief Forwarded to SX1278::finishTransmit().
    int16_t finishTransmit() override
    {
        return _radio->finishTransmit();
    }

    /// @brief Forwarded to SX1278::startReceive().
    int16_t startReceive() override
    {
        return _radio->startReceive();
    }

    /// @brief Forwarded to SX1278::getPacketLength().
    size_t getPacketLength() override
    {
        return _radio->getPacketLength();
    }

    /// @brief Forwarded to SX1278::readData().
    int16_t readData(uint8_t* data, size_t len) override
    {
        return _radio->readData(data, len);
    }

    /// @brief Forwarded to SX1278::getRSSI().
    float getRSSI() override { return _radio->getRSSI(); }

    /// @brief Forwarded to SX1278::getSNR().
    float getSNR()  override { return _radio->getSNR();  }

    /// @brief Forwarded to SX1278::sleep().
    int16_t sleep()   override { return _radio->sleep();   }

    /// @brief Forwarded to SX1278::standby().
    int16_t standby() override { return _radio->standby(); }

private:
    SX1278* _radio;   ///< Non-owning pointer to the SX1278 object (owned by LoraRadio::init()).
};
