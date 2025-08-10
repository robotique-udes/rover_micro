#ifndef MQ137_HPP
#define MQ137_HPP

#include "config.hpp"
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/log_plot.hpp"
#include "rover_can2/rover_can2.hpp"
#include "rover_can2/msgs/sensor_box .hpp"

#include <math.h>
#include <Wire.h>

DEFINE_LOG_NODE(GAS_SENSOR, Logger::eNodeState::OFF);

class GasSensor
{
    static constexpr uint8_t I2C_ADDR = 0x30;
    static constexpr uint8_t REG_VALUE0 = 0x00;  // 2-byte little-endian measurement
    static constexpr uint32_t I2C_HZ = 100'000UL;

    static constexpr uint16_t MAX_BITS = 2048;

    using GasSensorJointDeviceT = RoverCan2::Device<RoverCan2::Publisher<RoverCan2::Msgs::SensorBox>>;


  public:
    GasSensor(TwoWire& wire):
        _wire(wire)
    {
    }

    void init()
    {
        _wire.begin(MQ131_SDA, MQ131_SCL, I2C_HZ);
    }

    void update() 
    {
        _amonia = this->readMQ137();
        _hydrogen = this->toPercent(analogRead(MQ8_AOUT));
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

    GasSensorJointDeviceT& getGasSensorJointDevice()
    {
        return _gasSensorJointDevice;
    }

  private:
    void sendCamMsgs()
    {
        RoverCan2::Msgs::SensorBox sensorStatus;
        sensorStatus.data().amonia = _amonia;
        sensorStatus.data().hydrogen = _hydrogen;
    }

    float _amonia = 0.0F;
    float _hydrogen = 0.0F;

    TwoWire& _wire;

    GasSensorJointDeviceT _gasSensorJointDevice = 
        GasSensorJointDeviceT(RoverCan2::Constant::eDeviceId::GAS_SENSORS,
                              RoverCan2::Publisher<RoverCan2::Msgs::SensorBox>());
};

#endif
