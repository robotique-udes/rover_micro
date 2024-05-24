#ifndef __CUI_AMT222_HPP__
#define __CUI_AMT222_HPP__

#include "Arduino.h"
#include "SPI.h"

#include "rover_helpers/assert.hpp"
#include "rover_helpers/macros.hpp"

#include "sensors/encoders/encoder.hpp"

#if !defined(ESP32)
#error CPU is not supported
#else

class CUI_AMT222 : public Encoder
{
public:
    static constexpr uint16_t INVALID_POSITION = 0xFFFF;
    static constexpr uint8_t REG_START = 0x00;
    static constexpr uint8_t REG_READ = 0x00;
    static constexpr uint8_t REG_RESET = 0x60;
    static constexpr uint8_t REG_SET_ZERO = 0x70;

    /// @brief Constructor
    /// @param spiBus_ Must call SPI*.begin() before passing to encoder
    /// @param pinCs_ Chip select low pin
    CUI_AMT222(){}

    void init()
    {
        ASSERT(true, "CUI_AMT222 can't be initialized without arguments");
    }

    void init(SPIClass *spiBus_, gpio_num_t pinCs_)
    {
        ASSERT(spiBus_ == NULL);
        _pSpiBus = spiBus_;
        _pinCs = pinCs_;

        pinMode(_pinCs, OUTPUT);
        digitalWrite(_pinCs, HIGH);

        _inited = true;
    }

    float getPosition(void)
    {
        return this->readPosition();
    }

    float getSpeed(void)
    {
        ASSERT(true, "Method not implemented yet");
    }

    void setZero(void)
    {
        this->sendCmd(REG_SET_ZERO);
    }

    void reset(void)
    {
        this->sendCmd(REG_RESET);
    }

private:
    SPIClass *_pSpiBus = NULL;
    gpio_num_t _pinCs = GPIO_NUM_NC;
    bool _inited;

    bool isInited(void)
    {
        return _inited;
    }
    float readPosition()
    {
        ASSERT(!this->isInited());

        SPI.beginTransaction(SPISettings(2'000'000u, MSBFIRST, SPI_MODE0));
        digitalWrite(_pinCs, LOW);

        uint8_t positionByte0 = SPI.transfer(REG_START);
        uint8_t positionByte1 = SPI.transfer(REG_READ);

        uint16_t positionRaw = positionByte0 << 8 | positionByte1;
        positionRaw &= 0x3FFF;
        positionRaw = positionRaw >> 1;

        digitalWrite(_pinCs, HIGH);
        SPI.endTransaction();

        return MAP(positionRaw, 0, 2 << 12, 0.0f, 360.0f);
    }
    void sendCmd(uint8_t cmdRegister)
    {
        ASSERT(!this->isInited());

        SPI.beginTransaction(SPISettings(2'000'000u, MSBFIRST, SPI_MODE0));
        digitalWrite(_pinCs, LOW);

        SPI.transfer(REG_START);
        SPI.transfer(cmdRegister);

        digitalWrite(_pinCs, HIGH);
        SPI.endTransaction();
    }
};

#endif // !defined(ESP32)
#endif // __CUI_AMT222_HPP__
