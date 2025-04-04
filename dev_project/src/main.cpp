#include <Arduino.h>

#include "rover_lib2/helpers/macros.hpp"

#include "rover_can2/drivers/can_driver_mock.hpp"
#include "rover_can2/managers/slave_can_manager.hpp"

void setup()
{
    Serial.begin(115200);
    RoverCan2::Drivers::CanDriverMock canDriver;
    // RoverCan2::Managers::SlaveCanManager<RoverCan2::Drivers::CanDriverMock> canManager(canDriver);

    for (EVER)
    {
        uint8_t byte = Serial.read();
        Serial.write(byte);
    }
}

void loop() {}
