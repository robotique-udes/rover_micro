#include "Arduino.h"

#include "config_local.hpp"

#include "rover_helpers/helpers.hpp"
#include "systems/joint/dc_revolute_joint.hpp"
#include "actuators/motor_drivers/IFX007T.hpp"
#include "sensors/encoders/CUI_AMT222A_V.hpp"
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
    motor.enable();
    motor.setCmd(0.0f);
    motor.setMaxVoltage(25.2f, 25.2f, true);

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, GPIO_NUM_NC);
    CUI_AMT222A_V encoder(&SPI, PIN_SPI_CS_EN_SHAFT);
    encoder.init();

#warning TODO: Tuning
    PID pidPos(600.0f, 20.0f, 100.0f, 80.0f); 
    PID pidSpeed(20.0f, 2.0f, 0.0f, 50.0f);

    DcRevoluteJoint j2(&motor,
                       Encoder::eEncoderType::ABSOLUTE_SINGLE_TURN,
                       &encoder,
                       Joint::eControlMode::SPEED,
                       false,
                       &pidPos,
                       &pidSpeed);
    j2.setJointLimits(DEG_TO_RAD * -40.0f, DEG_TO_RAD * 40.0f);
    j2.init();

    LimitSwitch switchFWD(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_FWD);
    switchFWD.init();
    LimitSwitch switchREV(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_REV);
    switchREV.init();

    LimitSwitch switchCalib(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_CALIB);
    switchCalib.init();

    RoverHelpers::Timer<unsigned long, millis> timerFeedback(50);
    RoverHelpers::Timer<unsigned long, millis> timer(500);

    float goal = 0.0f;
    char cmd = 0;
    for (;;)
    {
        if (switchCalib.isClicked())
        {
            j2.calib(0.0f);
        }
        else
        {
            if (Serial.available() >= 1)
            {
                Serial.readBytes(&cmd, sizeof(cmd));

                if (cmd == '3')
                {
                    goal += 0.01;
                }
                else if (cmd == '1')
                {
                    goal -= 0.01;
                }
                else
                {
                    goal += goal > 0 ? -1.0f : 1.0f;
                }
            }

            j2.setSpeed(0.5f);
            j2.setPosition(goal);
        }

        j2.update();
        j2.printDebugInfo();
    }
}

void loop() {}
