#include <Arduino.h>
#include <rover_lib2/LED/led_blinker.hpp>

#include "config.hpp"
#include "J34Device.hpp"
#include "J5Device.hpp"
#include "rover_lib2/sensors/push_button.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"
#include "rover_can2/rover_can2.hpp"

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);
DEFINE_LOG_NODE(MainPlot, Logger::eNodeState::ON);

void setup(void)
{
#if defined(PCB_ROVER_J345_REV1)
    {
        // PWM pin is on a pin with default behavior (TX0) and is high by default, this cause the motor to glitch at startup. This
        // block helps limits its impact
        IO::DigitalOutput j34L_pwmPin(PIN_J34_R_PWM);
        j34L_pwmPin.write(IO::eIOState::LOW_);
    }
#endif

    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif

    // LED::LedBlinkerSoft statusLed(IO::DigitalOutput(PIN_USER_LED), LED::BlinkPatterns::HEARTBEAT);
    // statusLed.init();

    J34Device j34;
    j34.init();

    J5Device j5;
    j5.init();

    LED::LedBlinkerSoft canLed(IO::DigitalOutput(PIN_CAN_LED), LED::BlinkPatterns::HEARTBEAT);
    RoverCan2::Drivers::DriverESP32<LED::LedBlinkerSoft> canDriver(PIN_CAN_RX, PIN_CAN_TX, &canLed);
    RoverCan2::ManagerSlave canManager(canDriver, j34.getJ3Device(), j34.getJ4Device(), j5.getUnderlyingCanDevice());
    canManager.init();

    LOG_INFO(Logger::Nodes::Main, "J345 Init done, starting main loop!");
    for (EVER)
    {
        // statusLed.update();
        j34.update();
        j5.update();
        canManager.update();
    }
}

void loop()
{
    // Don't use
}
