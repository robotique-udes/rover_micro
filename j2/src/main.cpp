#include "Arduino.h"

#include "config_local.hpp"

#include "rover_helpers/helpers.hpp"
#include "systems/joint/dc_revolute_joint.hpp"
#include "actuators/motor_drivers/IFX007T.hpp"
#include "sensors/encoders/CUI_AMT222.hpp"
#include "rover_helpers/PID.hpp"
#include "sensors/limit_switch.hpp"
#include "SPI.h"

#warning TODO: Joint limits, PID "zero holding torque range", Faire PID dual band (peut-être modèle dynamique?)

void setup()
{
    Serial.begin(115200);
    delay(2000);

    LOG(WARN, "Init done starting!");

    IFX007T motor(PIN_J2_EN_1, PIN_J2_EN_2, PIN_J2_IN_1, PIN_J2_IN_2, MotorDriver::eBrakeMode::BRAKE, false);
    motor.init();
    motor.enable();
    motor.setCmd(0.0f);
    motor.setMaxVoltage(24.0f, 12.0f, false);

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, GPIO_NUM_NC);
    CUI_AMT222 encoder(&SPI, PIN_SPI_CS_EN_SHAFT);
    encoder.init();
#warning TODO, Should be #define
    PID pid(850.0f, 2000.0f, 0.0f, 10.0f);
    DcRevoluteJoint j2(&motor, &encoder, Joint::eControlMode::POSITION, &pid);
#warning TODO, Should be #define
    j2.setJointLimits(DEG_TO_RAD*-10.0f, DEG_TO_RAD*10.0f);
    j2.init();

    LimitSwitch switchFWD(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_FWD);
    switchFWD.init();
    LimitSwitch switchREV(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_REV);
    switchREV.init();

    LimitSwitch switchCalib(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_CALIB);
    switchCalib.init();

    RoverHelpers::Timer<unsigned long, millis> timerFeedback(50);
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

        j2.setPosition(DEG_TO_RAD*5.0f);
        j2.update();

        if (switchCalib.isClicked())
        {
            j2.calib(0.0f);
        }

        if (timerFeedback.isDone())
        {
            j2.printDebugInfo();
        }
    }
}

void loop() {}
