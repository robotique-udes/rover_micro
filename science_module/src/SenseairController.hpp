#ifndef SENSEAIR_CONTROLLER
#define SENSEAIR_CONTROLLER

#include <Wire.h>

#include "rover_lib2/helpers/loop_timer.hpp"

DEFINE_LOG_NODE(SenseAir, Logger::eNodeState::OFF);

class SenseairController
{
    enum class eSensorId : uint8_t
    {
        SENSOR_1 = 0,
        SENSOR_2,
        SENSOR_3
    };

    static constexpr uint64_t LOOP_PERIOD_US = 250ULL;
    static constexpr uint8_t ADDRESS_SENSOR_1 = 0x69;
    static constexpr uint8_t ADDRESS_SENSOR_2 = 0x70;
    static constexpr uint8_t ADDRESS_SENSOR_3 = 0x71;

    void init()
    {
        Wire.begin();
    }

    void update() {}

  private:
    int readCO2(int& CO2level, eSensorId id_)
    {
        uint8_t recValue[4] = {0, 0, 0, 0};

        uint8_t address = 0x00;

        switch (id_)
        {
            case eSensorId::SENSOR_1:
                address = ADDRESS_SENSOR_1;
                break;
            case eSensorId::SENSOR_2:
                address = ADDRESS_SENSOR_2;
                break;
            case eSensorId::SENSOR_3:
                address = ADDRESS_SENSOR_3;
                break;
            default:
                LOG_WARN(Logger::Nodes::SenseAir, "Unknown sensor.")
                return;
        }

        Wire.beginTransmission(address);
        Wire.write(0x22);
        Wire.write(0x00);
        Wire.write(0x08);
        Wire.write(0x2A);
        Wire.endTransmission();
        delay(30);

        Wire.requestFrom(address, 4);
        delay(20);

        uint8_t i = 0;
        while (Wire.available())
        {
            recValue[i] = Wire.read();
            i++;
        }

        uint8_t checkSum = recValue[0] + recValue[1] + recValue[2];
        CO2level = (recValue[1] << 8) + recValue[2];

        if (i == 0)
        {
            return 2;
        }
        else if (checkSum == recValue[3])
        {
            return 0;
        }
        else
        {
            return 1;
        }
    };

    LoopTimer<uint64_t, &Time::micros> _loopTimer = {LOOP_PERIOD_US};
};

#endif  // SENSEAIR_CONTROLLER