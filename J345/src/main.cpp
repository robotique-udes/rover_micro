#include <Arduino.h>
#include <rover_lib2/LED/led_blinker.hpp>

#include "config.hpp"
#include "J34Device.hpp"
#include "J5Actuator.hpp"
#include "rover_lib2/sensors/push_button.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);
DEFINE_LOG_NODE(MainPlot, Logger::eNodeState::ON);

void setup(void)
{
#if defined(PCB_ROVER_J345_REV1)
    {
        // PWM pin is on a pin with default behavior (TX0) and is high by default, this cause the motor to glitch at startup. This
        // block limits the impact of this
        IO::DigitalOutput j34L_pwmPin(PIN_J34_R_PWM);
        j34L_pwmPin.write(IO::eIOState::LOW_);
    }
#endif

    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif

    LED::LedBlinkerSoft led(IO::DigitalOutput(PIN_USER_LED), LED::BlinkPatterns::HEARTBEAT);
    led.init();

    J34Device j34;
    j34.init();

    J5Actuator j5;
    j5.init();

    LOG_INFO(Logger::Nodes::Main, "J345 Init done, starting main loop!");
    for (EVER)
    {
        led.update();
        j34.update();
        j5.update();
    }
}

void loop()
{
    // Don't use
}
