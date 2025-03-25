#include <Arduino.h>

#include "rover_can2/can_manager.hpp"
#include "rover_can2/drivers/can_driver_esp.hpp"
#include "rover_can2/can_device.hpp"

void setup()
{
    RoverCan2::CanDriverESP driver(GPIO_NUM_47, GPIO_NUM_48);

    // RoverCan2::CanManager manager(driver, )
}

void loop() {}
