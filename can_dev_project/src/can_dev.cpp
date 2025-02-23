#include "Arduino.h"

#include "config_local.hpp"
#include "rover_lib2/helpers/log.hpp"
// #include "rover_helpers/helpers.hpp"

#include "rover_can2/can_driver.hpp"

DEFINE_LOG_NODE(MotorController);
struct EnabledNodes : Logger::NodeFilter<Logger::Nodes::MotorController> {};

void setup()
{
    // Logger::g_loggerOutput = Serial;
    Serial.begin(115200UL);

    float a = 60.0f;
    while (true)
    {
        LOG::DEBUG("Nothing to see here... %.3f", a);
        LOG::INFO("Nothing to see here... %.3f", a);
        LOG::WARN("Nothing to see here... %.3f", a);
        LOG::ERROR("Nothing to see here... %.3f", a);
        LOG::FATAL("Nothing to see here... %.3f", a);
    }
}

void loop() {}
