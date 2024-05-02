#include "Arduino.h"

#include "config_local.hpp"

#include "rover_helpers/helpers.hpp"
#include "rover_can_lib/rover_can_lib.hpp"
#include "rover_can_lib/msgs/propulsion_motor_cmd.hpp"
#include "rover_can_lib/msgs/propulsion_motor_status.hpp"

#include "actuators/talon_srx.hpp"
#include "actuators/motor_driver.hpp"

#define PMW_MOT GPIO_NUM_26
#define BTN_1 GPIO_NUM_34
#define BTN_2 GPIO_NUM_39
#define BTN_3 GPIO_NUM_36

#define LED_1 GPIO_NUM_32
#define LED_2 GPIO_NUM_33
#define LED_3 GPIO_NUM_25

void canCB(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_);
void parseDeviceIdMsg(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_);

RoverCanLib::Msgs::PropulsionMotorCmd msgPropCmd;
RoverCanLib::Msgs::PropulsionMotorStatus msgPropStatus;

void setup()
{
    Serial.begin(115200);

    pinMode(PMW_MOT, OUTPUT);
    pinMode(BTN_1, INPUT);
    pinMode(BTN_2, INPUT);
    pinMode(BTN_3, INPUT);
    pinMode(LED_1, OUTPUT);
    pinMode(LED_2, OUTPUT);
    pinMode(LED_3, OUTPUT);

    TalonSrx talonDrive(PMW_MOT, LEDC_TIMER_0, LEDC_CHANNEL_0);
    talonDrive.init(1500.0f);

    RoverCanLib::CanBusManager canBus(DEVICE_ID, GPIO_NUM_4, GPIO_NUM_18, canCB, true, (gpio_num_t)LED_BUILTIN);
    canBus.init();

    Serial.printf("Init done, starting Loop!\n");
    RoverHelpers::Timer<unsigned long, millis> timerFeedback(100); // 10 Hz
    RoverHelpers::Timer<unsigned long, millis> timer500(100);
    float testSpeed = 0;
    // talonDrive.setMaxSpeed(64); // enlever par sécurité

    for (;;)
    {
        if (timer500.isDone())
        {
            digitalWrite(LED_1, !digitalRead(LED_1));
            talonDrive.setSpd(30);
        }

        // talonDrive.writeMicroseconds(MAP(testSpeed, -100.0f, 100.0f, 1000.0f, 2000.0f));
        // canBus.update();

        // if (canBus.isOk() && msgPropCmd.data.enable)
        // {
        // talonDrive.writeMicroseconds(MAP(msgPropCmd.data.targetSpeed, -100.0f, 100.0f, 1000.0f, 2000.0f));
        // }
        // else
        // {
        //     motorDrive.writeMicroseconds(TalonSrxConstant::SIGNAL_FULL_STOP_MS);
        // }

        // if (timerFeedback.isDone())
        // {
        //     if (!msgPropCmd.data.closeLoop)
        //     {
        //         msgPropStatus.data.currentSpeed = msgPropCmd.data.targetSpeed;
        //     }
        //     else
        //     {
        //         LOG(ERROR, "Close loop not implemented yet");
        //         canBus.sendErrorCode(RoverCanLib::Constant::eInternalErrorCode::ERROR);
        //         msgPropStatus.data.currentSpeed = -69.0f;
        //     }

        //     canBus.sendMsg(&msgPropStatus);
        // }
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
