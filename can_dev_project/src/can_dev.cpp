#include "Arduino.h"

#include "config_local.hpp"
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/macros.hpp"
// #include "rover_helpers/helpers.hpp"

#include "rover_can2/can_driver.hpp"

DEFINE_LOG_NODE(Main, Logger::eNodeState::OFF);

void setup()
{
    // Logger::loggerStream = ;
    Serial.begin(115200UL);
    delay(1000);

    RoverCan2::CanDriver driver(GPIO_NUM_47, GPIO_NUM_48);
    driver.init();

    for (EVER)
    {
        driver.update();
    }
}

void loop() {}
