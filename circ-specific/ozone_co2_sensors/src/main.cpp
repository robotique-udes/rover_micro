#include "Arduino.h"
#include <Wire.h>

#include "config.hpp"
#include "MQ137.hpp"
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/LED/led_blinker.hpp"
#include "rover_can2/rover_can2.hpp"

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

void setup(void)
{
    Serial.begin(115200);

    LED::LedBlinkerSoft canLed(IO::DigitalOutput(LED_BLTN), LED::BlinkPatterns::HEARTBEAT);
    canLed.init();
    
    MQ137 mq137(Wire1);
    mq137.init();

    LOG_INFO(Logger::Nodes::Main, "MQ137 Init done, starting main loop!");
    for (EVER)
    {
        canLed.update();
        mq137.update();
    }

}

void loop()
{
    // Don't use
}
