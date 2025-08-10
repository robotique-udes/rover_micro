#ifndef MQ137_HPP
#define MQ137_HPP

#include "config.hpp"
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/log_plot.hpp"
#include "rover_can2/rover_can2.hpp"

#include <math.h>
#include <Wire.h>

DEFINE_LOG_NODE(GAS_SENSOR, Logger::eNodeState::OFF);

class GAS_SENSORS
{
    static constexpr uint8_t I2C_ADDR = 0x30;
    static constexpr uint8_t REG_VALUE0 = 0x00;  // 2-byte little-endian measurement
    static constexpr uint32_t I2C_HZ = 100'000UL;

    static constexpr uint16_t MAX_BITS = 2048;


  public:
    GAS_SENSORS(TwoWire& wire):
        _wire(wire)
    {
    }

    void init()
    {
        _wire.begin(MQ131_SDA, MQ131_SCL, I2C_HZ);
    }

    void update() 
    {
        float amonia = this->readMQ137();
        float hydrogen = this->toPercent(analogRead(MQ8_AOUT));
    }

    float toPercent(float value)
    {
        return value / MAX_BITS * 100.0F;
    }

    float readMQ137()
    {
        _wire.beginTransmission(I2C_ADDR);
        _wire.write(REG_VALUE0);
        if (_wire.endTransmission(false) != 0)
        {
            return NAN;
        }

        if (_wire.requestFrom((int)I2C_ADDR, 2) != 2)
        {
            return NAN;
        }

        uint8_t lowByte = _wire.read();
        uint8_t highByte = _wire.read();
        uint16_t rawValue = (uint16_t(highByte) << 8) | lowByte;

        return static_cast<float>(rawValue);  // ppb
    }

  private:
    TwoWire& _wire;
};

#endif
