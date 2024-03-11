#include "Arduino.h"

#include "helpers/helpers.hpp"
#include "rover_can_lib/rover_can_lib.hpp"

#include "actuators/talon_srx.hpp"

#include "config_local.hpp"

#define PIN_PWM 23
#define PIN_GND 22

void canCB(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_);
void parseDeviceIdMsg(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_);

RoverCanLib::Msg::PropulsionMotor::sMsgData propMsgStruct;

void setup()
{
    Serial.begin(115200);

    RoverCanLib::CanBusManager canBus(DEVICE_ID, GPIO_NUM_4, GPIO_NUM_18, canCB, (gpio_num_t)LED_BUILTIN);
    canBus.init();

    pinMode(PIN_GND, OUTPUT);
    digitalWrite(PIN_GND, LOW);

    TalonSrx motorDrive(GPIO_NUM_23, LEDC_TIMER_0, LEDC_CHANNEL_0);
    motorDrive.init(1500.0f);

    Serial.printf("Init done, starting Loop!\n");

    for (;;)
    {
        canBus.update();

        if (canBus.isOk() && propMsgStruct.enable)
        {
            motorDrive.writeMicroseconds(MAP(propMsgStruct.targetSpeed, -100.0f, 100.0f, 1000.0f, 2000.0f));
        }
        else
        {
            motorDrive.writeMicroseconds(TalonSrxConstant::SIGNAL_FULL_STOP_MS);
        }
    }
}

void loop() {}

void canCB(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_)
{
    switch (msg_->identifier)
    {
    case (DEVICE_ID):
        if (msg_->data_length_code == 0)
        {
            LOG(WARN, "Ill formed msg, dropping");
            canBusManager_->setWarningFlag();
        }
        else
        {
            parseDeviceIdMsg(canBusManager_, msg_);
        }
        break;

    default:
        break;
    }
}

void parseDeviceIdMsg(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_)
{
    canBusManager_->resetWatchDog();

    switch (msg_->data[0])
    {
    case RoverCanLib::Msg::PropulsionMotor::eMsgID::CLOSE_LOOP:
        propMsgStruct.closeLoop = msg_->data[1];
        LOG(WARN, "Unsupported feature");
        canBusManager_->setWarningFlag();
        break;

    case RoverCanLib::Msg::PropulsionMotor::eMsgID::ENABLE:
        propMsgStruct.enable = msg_->data[1];

    case RoverCanLib::Msg::PropulsionMotor::eMsgID::TARGET_SPEED:
        RoverCanLib::Helpers::canMsgToStruct<float,
                                             RoverCanLib::UnionDefinition::FloatUnion>(&(msg_->data[1]),
                                                                                       &propMsgStruct.targetSpeed);
        break;

    case RoverCanLib::Msg::PropulsionMotor::eMsgID::KP:
        RoverCanLib::Helpers::canMsgToStruct<float,
                                             RoverCanLib::UnionDefinition::FloatUnion>(&(msg_->data[1]),
                                                                                       &propMsgStruct.kp);
        break;

    case RoverCanLib::Msg::PropulsionMotor::eMsgID::KI:
        RoverCanLib::Helpers::canMsgToStruct<float,
                                             RoverCanLib::UnionDefinition::FloatUnion>(&(msg_->data[1]),
                                                                                       &propMsgStruct.ki);
        break;

    case RoverCanLib::Msg::PropulsionMotor::eMsgID::KD:
        RoverCanLib::Helpers::canMsgToStruct<float,
                                             RoverCanLib::UnionDefinition::FloatUnion>(&(msg_->data[1]),
                                                                                       &propMsgStruct.kd);
        break;

    default:
        LOG(WARN, "Unknown \"Message Specific Id\"");
        canBusManager_->setWarningFlag();
    }
}
