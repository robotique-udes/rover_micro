#include "Arduino.h"
#include "EEPROM.h"
#include "config_local.hpp"

#include "rover_helpers/helpers.hpp"
#include "systems/joint/dc_revolute_joint.hpp"
#include "systems/joint/differential_joint.hpp"
#include "actuators/motor_drivers/DRV8251A.hpp"
#include "sensors/encoders/CUI_AMT222.hpp"
#include "rover_helpers/PID.hpp"
#include "sensors/limit_switch.hpp"

constexpr uint8_t ADDRESS_CALIB_DIFF_A = 0x10;
constexpr uint8_t ADDRESS_CALIB_DIFF_B = 0x20;

void setup()
{
    Serial.begin(115200);
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, GPIO_NUM_NC);

    // DRV8251A motorDiffA(PIN_DIFF_A_IN_1,
    //                     PIN_DIFF_A_IN_2,
    //                     MotorDriver::eBrakeMode::BRAKE,
    //                     true,
    //                     LEDC_TIMER_0,
    //                     LEDC_CHANNEL_0,
    //                     LEDC_CHANNEL_1);
    // motorDiffA.setMaxVoltage(25.2f, 5.0f, false);
    // CUI_AMT222 encoderDiffA(&SPI, PIN_CS_DIFF_A, true, Encoder::eEncoderType::ABSOLUTE_MULTI_TURN);
    // PID pidA(100'000.0f, 5.0f, 0.0f, 50.0f);

    // DcRevoluteJoint jointDiffA(&motorDiffA,
    //                            Encoder::eEncoderType::ABSOLUTE_MULTI_TURN,
    //                            &encoderDiffA,
    //                            Joint::eControlMode::POSITION,
    //                            false,
    //                            &pidA);
    // jointDiffA.init();
    // jointDiffA.update();
    // jointDiffA.setPIDDeadband(DEG_TO_RAD*0.005f, Joint::DEFAULT_PID_DEADBAND_RAD_SPEED);

    // DRV8251A motorDiffB(PIN_DIFF_B_IN_1,
    //                     PIN_DIFF_B_IN_2,
    //                     MotorDriver::eBrakeMode::BRAKE,
    //                     true,
    //                     LEDC_TIMER_0,
    //                     LEDC_CHANNEL_2,
    //                     LEDC_CHANNEL_3);
    // motorDiffB.setMaxVoltage(25.2f, 5.0f, false);
    // CUI_AMT222 encoderDiffB(&SPI, PIN_CS_DIFF_A, true, Encoder::eEncoderType::ABSOLUTE_MULTI_TURN);
    // PID pidB(100.0f, 0.0f, 0.0f, 0.0f);

    // DcRevoluteJoint jointDiffB(&motorDiffB,
    //                            Encoder::eEncoderType::ABSOLUTE_MULTI_TURN,
    //                            &encoderDiffB,
    //                            Joint::eControlMode::POSITION,
    //                            false,
    //                            &pidB);
    // jointDiffB.init();
    // jointDiffB.update();

    // DifferentialJoint differential(&jointDiffA, &jointDiffB, Joint::eControlMode::POSITION);
    // differential.init();

    LimitSwitch SW_diffUp(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_DIFF_UP);
    SW_diffUp.init();
    // LimitSwitch SW_diffDown(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_DIFF_DOWN);
    // SW_diffDown.init();
    // LimitSwitch SW_diffRotL(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_DIFF_LEFT);
    // SW_diffRotL.init();
    // LimitSwitch SW_diffRotR(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_DIFF_RIGHT);
    // SW_diffRotR.init();
    LimitSwitch SW_gripClose(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_GRIP_FWD);
    SW_gripClose.init();
    LimitSwitch SW_gripOpen(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_GRIP_REV);
    SW_gripOpen.init();

    LOG(WARN, "Init done starting!");
    for (;;)
    {
        jointDiffB.update();
        jointDiffB.setPosition(2.0f);
        jointDiffB.printDebugInfo();
        // differential.update();
        // differential.printDebugInfo();
        // Missing a calib button
        if (SW_gripClose.isClicked() && SW_gripOpen.isClicked())
        {
            float positionOffsetA = -jointDiffA.getPosition(true);
            // float positionOffsetB = -jointDiffB.getPosition(true);

            encoderDiffA.calib(positionOffsetA, true);
            // encoderDiffB.calib(positionOffsetB, true);

            delay(1000);
        }
        else if (SW_diffUp.isClicked())
        {
            motorDiffA.setCmd(-100.0f);
            // motorDiffB.setCmd(-100.0f);
        }
        // else if (SW_diffDown.isClicked())
        // {
        //     motorDiffA.setCmd(100.0f);
        //     motorDiffB.setCmd(100.0f);
        // }
        // else if (SW_diffRotL.isClicked())
        // {
        //     motorDiffA.setCmd(-100.0f);
        //     motorDiffB.setCmd(100.0f);
        // }
        // else if (SW_diffRotR.isClicked())
        // {
        //     motorDiffA.setCmd(100.0f);
        //     motorDiffB.setCmd(-100.0f);
        // }
        // else
        // {
        //     motorDiffA.setCmd(0.0f);
        //     motorDiffB.setCmd(0.0f);
        // }

        // LOG(INFO, "UP/DOWN: %f | ROT: %f", (encoderDiffA.getPosition() - encoderDiffB.getPosition())/2, (encoderDiffA.getPosition() + encoderDiffB.getPosition())/2);
        // LOG(INFO, "0: %f | 1: %f", encoderDiffA.getPosition() - encoderDiffB.getPosition());
    }
}

void loop() {}
