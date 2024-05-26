#ifndef __CUI_AMT222_HPP__
#define __CUI_AMT222_HPP__

#include "Arduino.h"
#include "SPI.h"

#include "rover_helpers/assert.hpp"
#include "rover_helpers/macros.hpp"
#include "rover_helpers/moving_average.hpp"

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
    // Will report errors if there's more than x% of error in msgs 
    static constexpr float ERROR_THRESHOLD = 0.10f;

    /// @brief Constructor
    /// @param spiBus_ Must call SPI*.begin() before passing to encoder
    /// @param pinCs_ Chip select low pin
    CUI_AMT222(SPIClass *spiBus_, gpio_num_t pinCs_)
    {
        ASSERT(spiBus_ == NULL);
        _pSpiBus = spiBus_;
        _pinCs = pinCs_;
    }

    ~CUI_AMT222(){};

    void init()
    {
        pinMode(_pinCs, OUTPUT);
        digitalWrite(_pinCs, HIGH);

        this->initDone();
    }

    void update() {}

    float getPosition(void)
    {
        return this->readPosition();
    }

    float getSpeed(void)
    {
        ASSERT(true, "Method not implemented yet");
    }

    void calib(float zeroPosition = 0.0f)
    {
        this->sendCmd(REG_SET_ZERO);

        ASSERT(zeroPosition >= 360.0f && zeroPosition < 0.0f);
        _positionCalibOffset = zeroPosition;
    }

    void reset(void)
    {
        this->sendCmd(REG_RESET);
    }

private:
    SPIClass *_pSpiBus = NULL;
    gpio_num_t _pinCs = GPIO_NUM_NC;
    float _positionCalibOffset = 0.0f;
    float _currentPosition = 0.0f;
    RoverHelpers::MovingAverage<bool, 10> _errorAvg = RoverHelpers::MovingAverage<bool, 10>(false);

    float readPosition()
    {
        this->checkInit();

        SPI.beginTransaction(SPISettings(2'000'000u, MSBFIRST, SPI_MODE0));
        digitalWrite(_pinCs, LOW);

        uint8_t positionByte0 = SPI.transfer(REG_START);
        uint8_t positionByte1 = SPI.transfer(REG_READ);

        uint16_t positionRaw = positionByte0 << 8 | positionByte1;

        // Checksum, see datasheet for infos
        bool bits[16] = {0};
        for (int i = 0; i < 16; i++)
        {
            bits[i] = (0x01) & (positionRaw >> (i));
        }

        if ((bits[15] == !(bits[13] ^ bits[11] ^ bits[9] ^ bits[7] ^ bits[5] ^ bits[3] ^ bits[1])) &&
            (bits[14] == !(bits[12] ^ bits[10] ^ bits[8] ^ bits[6] ^ bits[4] ^ bits[2] ^ bits[0])))
        {
            positionRaw &= 0x3FFF;
            positionRaw = positionRaw >> 1;

            LOG(INFO, "positionRaw: %u", positionRaw);
            _currentPosition = MAP(positionRaw, 0, 2 << 12, 0.0f, TWO_PI);
            _errorAvg.addValue(false);
        }
        else
        {
            // Checksum failed
            LOG(INFO, "Encoder read checksum failed, returning last valid position instead");
            if (_errorAvg.addValue(true) > ERROR_THRESHOLD);
        }

        digitalWrite(_pinCs, HIGH);
        SPI.endTransaction();

        return _currentPosition;
    }
    void sendCmd(uint8_t cmdRegister)
    {
        this->checkInit();

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
