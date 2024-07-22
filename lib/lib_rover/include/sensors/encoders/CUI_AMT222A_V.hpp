#ifndef __CUI_AMT222A_V_HPP__
#define __CUI_AMT222A_V_HPP__

#include "Arduino.h"
#include "SPI.h"

#include "rover_helpers/assert.hpp"
#include "rover_helpers/macros.hpp"
#include "rover_helpers/moving_average.hpp"
#include "rover_helpers/chrono.hpp"

#include "sensors/encoders/encoder.hpp"

#if !defined(ESP32)
#error CPU is not supported
#else

class CUI_AMT222A_V : public Encoder
{
public:
    static constexpr uint16_t INVALID_POSITION = 0xFFFF;
    static constexpr uint8_t REG_START = 0x00;
    static constexpr uint8_t REG_READ = 0x00;
    static constexpr uint8_t REG_RESET = 0x60;
    static constexpr uint8_t REG_SET_ZERO = 0x70;
    // Will report errors if there's more than x% of error in msgs
    static constexpr float ERROR_THRESHOLD = 0.50f;
    // 40us minimum as per datasheet
    static constexpr unsigned long TIME_BETWEEN_READ_US = 50ul;
    // 1 Hz should be fast enough
    static constexpr unsigned long TIME_SPEED_CALC_US = 10'000ul;

    /// @brief Constructor
    /// @param spiBus_ Must call SPI*.begin() before passing to encoder
    /// @param pinCs_ Chip select low pin
    CUI_AMT222A_V(SPIClass *spiBus_, gpio_num_t pinCs_, bool reverse_ = false);
    ~CUI_AMT222A_V(){};

    void init(void);

    void updateInternal(void);

    float getPositionInternal(void);

    float getSpeedInternal(void);

    void calib(float zeroPosition_ = 0.0f);

    void reset(void);

private:
    SPIClass *_pSpiBus = NULL;
    gpio_num_t _pinCs = GPIO_NUM_NC;
    float _positionCalibOffset = 0.0f;
    float _currentPosition = 0.0f;
    float _lastPosition = 0.0f;
    float _currentSpeed = 0.0f;
    RoverHelpers::MovingAverage<float, 10> _positionAvg = RoverHelpers::MovingAverage<float, 10>(0.0f);
    RoverHelpers::MovingAverage<float, 10> _speedAvg = RoverHelpers::MovingAverage<float, 10>(0.0f);

    RoverHelpers::Chrono<unsigned long, micros> _chronoSpeedCalc;
    RoverHelpers::MovingAverage<bool, 10> _errorAvg = RoverHelpers::MovingAverage<bool, 10>(false);
    RoverHelpers::Timer<unsigned long, micros> _timerRead = RoverHelpers::Timer<unsigned long, micros>(TIME_BETWEEN_READ_US);
    RoverHelpers::Timer<unsigned long, micros> _timerSpeedCalc = RoverHelpers::Timer<unsigned long, micros>(TIME_SPEED_CALC_US);

    float readPosition(void);
    void sendCmd(uint8_t cmdRegister);
};

CUI_AMT222A_V::CUI_AMT222A_V(SPIClass *spiBus_, gpio_num_t pinCs_, bool reverse_) : Encoder(reverse_)
{
    ASSERT(spiBus_ == NULL);
    _pSpiBus = spiBus_;
    _pinCs = pinCs_;
}

void CUI_AMT222A_V::init(void)
{
    pinMode(_pinCs, OUTPUT);
    digitalWrite(_pinCs, HIGH);

    this->initDone();
}

void CUI_AMT222A_V::updateInternal(void)
{
    _currentPosition = _positionAvg.addValue(CONSTRAIN_ANGLE(this->readPosition() + _positionCalibOffset));

    if (_timerSpeedCalc.isDone())
    {
        _currentSpeed = _speedAvg.addValue((_lastPosition - _currentPosition) / ((float)_chronoSpeedCalc.getTime() / 1'000'000.0f));
        _chronoSpeedCalc.restart();
        _lastPosition = _currentPosition;
    }
}

float CUI_AMT222A_V::getPositionInternal(void)
{
    return _currentPosition;
}

float CUI_AMT222A_V::getSpeedInternal(void)
{
    return _currentSpeed;
}

void CUI_AMT222A_V::calib(float zeroPosition_)
{
    this->sendCmd(REG_SET_ZERO);

    ASSERT(zeroPosition_ >= 360.0f && zeroPosition_ < 0.0f);
    _positionCalibOffset = zeroPosition_;
}

void CUI_AMT222A_V::reset(void)
{
    this->sendCmd(REG_RESET);
}

float CUI_AMT222A_V::readPosition(void)
{
    this->checkInit();

    if (_timerRead.isDone())
    {
        SPI.beginTransaction(SPISettings(2'000'000u, MSBFIRST, SPI_MODE0));
        digitalWrite(_pinCs, LOW);

        uint8_t positionByte0 = SPI.transfer(REG_START);
        uint8_t positionByte1 = SPI.transfer(REG_READ);

        uint16_t positionRaw = positionByte0 << 8 | positionByte1;

        float position = 0.0f;
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

            position = MAP(positionRaw, 0, 2 << 12, 0.0f, TWO_PI);
            _errorAvg.addValue(false);
        }
        else
        {
            // Checksum failed
            if (_errorAvg.addValue(true) > ERROR_THRESHOLD)
            {
                LOG(WARN, "Encoder read checksum failed, returning last valid values instead");
            }
            position = _currentPosition;
        }

        digitalWrite(_pinCs, HIGH);
        SPI.endTransaction();

        return position;
    }
    else
    {
        return _currentPosition;
    }
}
void CUI_AMT222A_V::sendCmd(uint8_t cmdRegister)
{
    this->checkInit();

    SPI.beginTransaction(SPISettings(2'000'000u, MSBFIRST, SPI_MODE0));
    digitalWrite(_pinCs, LOW);

    SPI.transfer(REG_START);
    SPI.transfer(cmdRegister);

    digitalWrite(_pinCs, HIGH);
    SPI.endTransaction();
}

#endif // !defined(ESP32)
#endif // __CUI_AMT222A_V_HPP__
