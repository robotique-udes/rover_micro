#include <Arduino.h>
#include <rover_lib2/LED/led_blinker.hpp>

#include "config.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_can2/rover_can2.hpp"
#include "JLDevice.hpp"

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);
DEFINE_LOG_NODE(MainPlot, Logger::eNodeState::ON);

void setup(void)
{
    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif

    JLDevice jL;
    jL.init();

    LED::LedBlinkerSoft canLed(IO::DigitalOutput(PIN_CAN_LED), LED::BlinkPatterns::HEARTBEAT);
    RoverCan2::Drivers::DriverESP32<LED::LedBlinkerSoft> canDriver(PIN_CAN_RX, PIN_CAN_TX, &canLed);
    RoverCan2::ManagerSlave canManager(canDriver);
    canManager.init();

    LOG_INFO(Logger::Nodes::Main, "JL Init done, starting main loop!");
    for (EVER)
    {
        canManager.update();
        jL.update();
    }
}

void loop()
{
    // Don't use
}
