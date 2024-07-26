#include "Arduino.h"

#include "config_local.hpp"

#include "rover_helpers/helpers.hpp"
#include "systems/joint/dc_revolute_joint.hpp"
#include "actuators/motor_drivers/DRV8251A.hpp"
#include "sensors/encoders/CUI_AMT222.hpp"
#include "rover_helpers/PID.hpp"
#include "sensors/limit_switch.hpp"
#include "SPI.h"

void setup()
{
    Serial.begin(115200);
    delay(2000);

    LOG(WARN, "Init done starting!");

    DRV8251A motorDiffA(PIN_DIFF_B_IN_1,
                        PIN_DIFF_B_IN_2,
                        MotorDriver::eBrakeMode::BRAKE,
                        false,
                        LEDC_TIMER_0,
                        LEDC_CHANNEL_0,
                        LEDC_CHANNEL_1);
    motorDiffA.init();
    motorDiffA.enable();
    motorDiffA.setCmd(20.0f);
    motorDiffA.setMaxVoltage(25.2f, 25.2f, true);

    DRV8251A motorDiffB(PIN_DIFF_A_IN_1,
                        PIN_DIFF_A_IN_2,
                        MotorDriver::eBrakeMode::BRAKE,
                        false,
                        LEDC_TIMER_0,
                        LEDC_CHANNEL_2,
                        LEDC_CHANNEL_3);
    motorDiffB.init();
    motorDiffB.enable();
    motorDiffB.setCmd(20.0f);
    motorDiffB.setMaxVoltage(25.2f, 25.2f, true);

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, GPIO_NUM_NC);
    CUI_AMT222 encoderDiffA(&SPI, PIN_CS_DIFF_A, false, Encoder::eEncoderType::ABSOLUTE_MULTI_TURN);
    encoderDiffA.init();


    for (;;)
    {
        motorDiffA.setCmd(100.0f);
        motorDiffA.update();
        motorDiffB.setCmd(100.0f);
        motorDiffB.update();
        
        encoderDiffA.update();
        // LOG(INFO, "Current Position is: %f", encoderDiffA.getPosition());
    }
}

void loop() {}
