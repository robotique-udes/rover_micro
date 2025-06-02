#include <Arduino.h>

#include "device_config.hpp"

#include "rover_can2/rover_can2.hpp"
#include "gimbal_controller.hpp"

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

void setup()
{
    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif  // defined(DEBUG)

    LED::LedBlinkerSoft userLED(IO::DigitalOutput(PIN_LED_4), LED::BlinkPatterns::HEARTBEAT);
    userLED.init();

    GimbalController gimbal;
    gimbal.init();

    LED::LedBlinkerSoft canLED(IO::DigitalOutput(PIN_CAN_LED), LED::BlinkPatterns::ON);
    RoverCan2::Drivers::DriverESP32<LED::LedBlinkerSoft> canDriver(PIN_CAN_RX, PIN_CAN_TX, &canLED);
    RoverCan2::ManagerSlave canManager(canDriver, gimbal);
    canManager.init();

    LOG_INFO(Logger::Nodes::Main, "Init done starting");
    for (EVER)
    {
        userLED.update();
        gimbal.update();
        canManager.update();
    }
}
