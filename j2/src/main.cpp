#include "Arduino.h"

#include "config_local.hpp"

#include "rover_helpers/helpers.hpp"
#include "actuators/IFX007T.hpp"

void setup()
{
    Serial.begin(115200);
    delay(2000);

    LOG(WARN, "Init done starting!");

    IFX007T motorDriver;
    motorDriver.init(PIN_J2_EN_1, PIN_J2_EN_2, PIN_J2_IN_1, PIN_J2_IN_2, MotorDriver::eBrakeMode::COAST, false);
    motorDriver.attachRGBLed(GPIO_NUM_3, GPIO_NUM_9, GPIO_NUM_10);
    motorDriver.enable();
    motorDriver.setSpeed(60.0f);

    RoverHelpers::Timer<unsigned long, millis> timer(2000);
    for (;;)
    {
        if (timer.isDone())
        {
            motorDriver.setSpeed(35.0f);
        }
    }
}

void loop() {}
