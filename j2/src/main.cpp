#include "Arduino.h"

#include "config_local.hpp"

#include "rover_helpers/helpers.hpp"
#include "actuators/IFX007T.hpp"
#include "sensors/limit_switch.hpp"

void setup()
{
    Serial.begin(115200);
    delay(2000);

    LOG(WARN, "Init done starting!");

    IFX007T motorDriver;
    motorDriver.init(PIN_J2_EN_1, PIN_J2_EN_2, PIN_J2_IN_1, PIN_J2_IN_2, MotorDriver::eBrakeMode::COAST, false);
    motorDriver.attachRGBLed(PIN_LED_R, PIN_LED_G, PIN_LED_B);
    motorDriver.enable();
    motorDriver.setSpeed(0.0f);

    LimitSwitch switchFWD;
    switchFWD.init(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_FWD);
    LimitSwitch switchREV;
    switchREV.init(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_REV);

    RoverHelpers::Timer<unsigned long, millis> timer(500);
    int16_t i = 0;
    for (;;)
    {
        if (switchFWD.isClicked())
        {
            motorDriver.setSpeed(100.0f);    
        }
        else if (switchREV.isClicked())
        {
            motorDriver.setSpeed(-100.0f);
        }
        else
        {
            motorDriver.setSpeed(0.0f);
        }
        motorDriver.update();
    }
}

void loop() {}
