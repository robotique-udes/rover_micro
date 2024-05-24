#include "Arduino.h"

#include "config_local.hpp"

#include "rover_helpers/helpers.hpp"
#include "systems/simple_dc_revolute_joint.hpp"
#include "actuators/motor_drivers/IFX007T.hpp"
#include "sensors/encoders/CUI_AMT222.hpp"
#include "rover_helpers/PID.hpp"
#include "sensors/limit_switch.hpp"
#include "SPI.h"

void setup()
{
    Serial.begin(115200);
    delay(2000);

    LOG(WARN, "Init done starting!");

    IFX007T motor(PIN_J2_EN_1, PIN_J2_EN_2, PIN_J2_IN_1, PIN_J2_IN_2, MotorDriver::eBrakeMode::BRAKE, false);
    motor.init();
    motor.attachRGBLed(PIN_LED_R, PIN_LED_G, PIN_LED_B);
    motor.enable();
    motor.setCmd(0.0f);

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, GPIO_NUM_NC);
    CUI_AMT222 encoder(&SPI, PIN_SPI_CS_EN_SHAFT);
    encoder.init();

    PID pid(0.0f, 10.0f, 0.0f, 100.0f);
    SimpleDcRevoluteJoint j2(&motor, &encoder, Joint::eControlMode::POSITION, &pid);

    LimitSwitch switchFWD(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_FWD);
    switchFWD.init();
    LimitSwitch switchREV(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_REV);
    switchREV.init();

    LimitSwitch switchCalib(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_CALIB);
    switchCalib.init();


    RoverHelpers::Timer<unsigned long, millis> timerFeedback(250);
    RoverHelpers::Timer<unsigned long, millis> timer(500);
    int16_t i = 0;
    for (;;)
    {
        if (switchFWD.isClicked())
        {
            motor.setCmd(100.0f);
            continue;
        }
        else if (switchREV.isClicked())
        {
            motor.setCmd(-100.0f);
            continue;
        }

        j2.setPosition(DEG_TO_RAD*140.0f);
        j2.update();

        if (switchCalib.isClicked())
        {
            #warning TODO: Pass by joint object
            encoder.calib();
        }

        if (timerFeedback.isDone())
        {
            LOG(INFO, "Current speed: %f | Current position: %f | Goal position: %f", motor.getCmd(), encoder.getPosition(), DEG_TO_RAD*140.0f);
        }
    }
}

void loop() {}
