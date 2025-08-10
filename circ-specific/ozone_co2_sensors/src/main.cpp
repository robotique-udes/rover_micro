#include "Arduino.h"
#include <Wire.h>

#include "config.hpp"
#include "gas_sensor.hpp"
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/LED/led_blinker.hpp"
#include "rover_can2/rover_can2.hpp"

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

void setup(void)
{
    Serial.begin(115200);

    LED::LedBlinkerSoft canLed(IO::DigitalOutput(LED_BLTN), LED::BlinkPatterns::HEARTBEAT);
    canLed.init();
    
    GAS_SENSORS gasSensor(Wire1);
    gasSensor.init();

    LOG_INFO(Logger::Nodes::Main, "gasSensor Init done, starting main loop!");
    for (EVER)
    {
        canLed.update();
        LOG_INFO(Logger::Nodes::Main, "MQ8 Analog Read: %d", anal);

        // 0.88 voltage at 645
    }

}

void loop()
{
    // Don't use
}
