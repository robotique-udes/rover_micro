#include "Arduino.h"

#include "config_local.hpp"

#include "rover_can_lib/can_bus_manager.hpp"
#include "rover_can_lib/msgs/arm_cmd.hpp"
#include "rover_can_lib/msgs/arm_status.hpp"

#include "rover_helpers/helpers.hpp"
#include "systems/joint/dc_revolute_joint.hpp"
#include "actuators/motor_drivers/IFX007T.hpp"
#include "sensors/encoders/CUI_AMT222.hpp"
#include "rover_helpers/PID.hpp"
#include "sensors/limit_switch.hpp"
#include "SPI.h"

void CB_Can(RoverCanLib::CanBusManager *canManager_, const twai_message_t *msgPtr_);

float g_goal = 0.0f;

void setup()
{
    Serial.begin(115200);
    LOG(WARN, "Init done starting!");

    IFX007T motor(PIN_JL_EN_1, PIN_JL_EN_2, PIN_JL_IN_1, PIN_JL_IN_2, MotorDriver::eBrakeMode::BRAKE, false);
    motor.init();
    motor.enable();
    motor.setCmd(0.0f);
    motor.setMaxVoltage(25.2f, 12.0f, true);

    LimitSwitch switchRight(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_RIGHT);
    switchRight.init();
    LimitSwitch switchLeft(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_LEFT);
    switchLeft.init();

    RoverHelpers::MovingAverage<float, 10> avgCmd;

    RoverCanLib::CanBusManager canBus(DEVICE_ID, PIN_RXC, PIN_TXC, CB_Can, false);
    canBus.init();

    RoverHelpers::Timer<unsigned long, millis> timerAvg(10);
    RoverHelpers::Timer<unsigned long, millis> timerFeedback(50);
    RoverHelpers::Timer<unsigned long, millis> timer(500);

    for (;;)
    {
        canBus.update();
        motor.update();

        if (timerAvg.isDone())
        {
            float cmd = 0.0f;
            if (canBus.isOk())
            {
                if (g_goal > 0.1f)
                {
                    LOG(INFO, "Here!");
                    cmd = 30.0f;
                }
                else if (g_goal < -0.1f)
                {
                    cmd = -30.0f;
                }
            }

            if (switchRight.isClicked())
            {
                cmd = constrain(cmd, 0.0f, 100.0f);
            }
            else if (switchLeft.isClicked())
            {
                cmd = constrain(cmd, -100.0f, 0.0f);
            }

            motor.setCmd(avgCmd.addValue(cmd));

            if (timerFeedback.isDone())
            {
                RoverCanLib::Msgs::armStatus msg;
                msg.data.currentSpeed = 0.0f;
                canBus.sendMsg(&msg);
            }
        }
    }
}

void loop() {}

void CB_Can(RoverCanLib::CanBusManager *canManager_, const twai_message_t *msgPtr_)
{
    if (msgPtr_->identifier == (uint32_t)DEVICE_ID)
    {
        canManager_->resetWatchDog();
        RoverCanLib::Msgs::armCmd armMsg;
        canManager_->sendErrorCode(armMsg.parseMsg(msgPtr_));

        LOG(INFO, "armMsg.data.targetSpeed: %f", armMsg.data.targetSpeed);

        g_goal = armMsg.data.targetSpeed * 1000.0f;
    }
}
