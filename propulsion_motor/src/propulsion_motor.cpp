#include "Arduino.h"

#include "config_local.hpp"

#include "rover_helpers/helpers.hpp"
#include "rover_can_lib/rover_can_lib.hpp"
#include "rover_can_lib/msgs/propulsion_motor_cmd.hpp"
#include "rover_can_lib/msgs/propulsion_motor_status.hpp"

#include "actuators/talon_srx.hpp"
#include "actuators/motor_driver.hpp"

void canCB(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_);
void parseDeviceIdMsg(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_);

RoverCanLib::Msgs::PropulsionMotorCmd msgPropCmd;
RoverCanLib::Msgs::PropulsionMotorStatus msgPropStatus;

void setup()
{
    Serial.begin(115200);

    pinMode(PWM_MOT, OUTPUT);

    TalonSrx talonDrive(PWM_MOT, LEDC_TIMER_0, LEDC_CHANNEL_0);
    talonDrive.init();

    RoverCanLib::CanBusManager canBus(DEVICE_ID, CAN_TX, CAN_RX, canCB, true, (gpio_num_t)LED_BUILTIN);
    canBus.init();

    LOG(INFO, "Init done, starting Loop!");
    RoverHelpers::Timer<unsigned long, millis> timerFeedback(100); // 10 Hz
    RoverHelpers::Timer<unsigned long, millis> timerSetCmd(10);

    RoverHelpers::MovingAverage<float, 35u> cmdAverage(0.0f);

    float speedCmd = 0.0f;
    for (;;)
    {
        canBus.update();

        if (timerSetCmd.isDone())
        {
            if (canBus.isOk() && msgPropCmd.data.enable)
            {
                if (DEVICE_ID == (uint16_t)RoverCanLib::Constant::eDeviceId::REARRIGHT_MOTOR || DEVICE_ID == (uint16_t)RoverCanLib::Constant::eDeviceId::FRONTRIGHT_MOTOR)
                {
                    speedCmd = cmdAverage.addValue(msgPropCmd.data.targetSpeed * -100.0f);
                }
                else
                {
                    speedCmd = cmdAverage.addValue(msgPropCmd.data.targetSpeed * 100.0f);
                }
            }
            else
            {
                speedCmd = cmdAverage.addValue(0.0f);
            }

            talonDrive.setCmd(speedCmd);
        }

        if (timerFeedback.isDone())
        {
            if (!msgPropCmd.data.closeLoop)
            {
                msgPropStatus.data.currentSpeed = msgPropCmd.data.targetSpeed;
            }
            else
            {
                LOG(ERROR, "Close loop not implemented yet");
                canBus.sendErrorCode(RoverCanLib::Constant::eInternalErrorCode::ERROR);
                msgPropStatus.data.currentSpeed = -69.0f;
            }

            canBus.sendMsg(&msgPropStatus);
        }
    }
}

void loop() {}

void canCB(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_)
{
    switch (msg_->identifier)
    {
    case (DEVICE_ID):
        if (msg_->data_length_code < 3)
        {
            LOG(WARN, "Ill formed msg, dropping");
            canBusManager_->sendErrorCode(RoverCanLib::Constant::eInternalErrorCode::WARNING);
            return;
        }
        else
        {
            canBusManager_->resetWatchDog();
            canBusManager_->sendErrorCode(msgPropCmd.parseMsg(msg_));
        }
        break;

    default:
        break;
    }
}

void parseDeviceIdMsg(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_)
{
    canBusManager_->resetWatchDog();
}
