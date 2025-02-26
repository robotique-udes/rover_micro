#include "rover_can2/drivers/can_driver_esp.hpp"
#include "rover_can2/can_device.hpp"

#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/chrono.hpp"

#include "rover_lib2/helpers/static_array.hpp"

#include <Arduino.h>
#include <functional>

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

void setup()
{
    Serial.begin(115200UL);
    delay(1000);

    Serial.print("Main Started");

    // RoverCan2::CanDriver driver(GPIO_NUM_47, GPIO_NUM_48);

    for (EVER)
    {
        // driver.update();
    }
}

void loop() {}
