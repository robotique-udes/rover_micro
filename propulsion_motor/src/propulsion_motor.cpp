#include "Arduino.h"
#include "config.hpp"

#include "helpers/helpers.hpp"
#include "rover_can_lib/can_bus_manager.hpp"
#include "rover_can_lib/propulsion_motor_msg.hpp"
#include "rover_can_lib/union_type_definition.hpp"
#include "rover_can_lib/helpers.hpp"

#include "actuators/talon_srx.hpp"
#include "driver/twai.h"

#define PIN_PWM 23
#define PIN_GND 22

void canCB(CanBusManager *canBusManager_, const twai_message_t *msg_);
void parseDeviceIdMsg(CanBusManager *canBusManager_, const twai_message_t *msg_);

PropulsionMotorMsg::sMsgData propMsgStruct;

void setup()
{
    Serial.begin(115200);

    CanBusManager canBus(DEVICE_ID, GPIO_NUM_4, GPIO_NUM_18, canCB, (gpio_num_t)LED_BUILTIN);
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

void canCB(CanBusManager *canBusManager_, const twai_message_t *msg_)
{
    switch (msg_->identifier)
    {
    case (DEVICE_ID):
        if (msg_->data_length_code == 0)
        {
            LOG(WARN, "I'll formed msg, dropping");
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

void parseDeviceIdMsg(CanBusManager *canBusManager_, const twai_message_t *msg_)
{
    canBusManager_->resetWatchDog();

    switch (msg_->data[0])
    {
    case PropulsionMotorMsg::eMsgID::CLOSE_LOOP:
        propMsgStruct.closeLoop = msg_->data[1];
        LOG(WARN, "Unsupported feature");
        canBusManager_->setWarningFlag();
        break;

    case PropulsionMotorMsg::eMsgID::ENABLE:
        propMsgStruct.enable = msg_->data[1];

    case PropulsionMotorMsg::eMsgID::TARGET_SPEED:
        RoverCanLib::Helpers::canMsgToStruct<float,
                                             RoverCanLib::UnionDefinition::FloatUnion>(&(msg_->data[1]),
                                                                                       &propMsgStruct.targetSpeed);
        break;

    case PropulsionMotorMsg::eMsgID::KP:
        RoverCanLib::Helpers::canMsgToStruct<float,
                                             RoverCanLib::UnionDefinition::FloatUnion>(&(msg_->data[1]),
                                                                                       &propMsgStruct.kp);
        break;

    case PropulsionMotorMsg::eMsgID::KI:
        RoverCanLib::Helpers::canMsgToStruct<float,
                                             RoverCanLib::UnionDefinition::FloatUnion>(&(msg_->data[1]),
                                                                                       &propMsgStruct.ki);
        break;

    case PropulsionMotorMsg::eMsgID::KD:
        RoverCanLib::Helpers::canMsgToStruct<float,
                                             RoverCanLib::UnionDefinition::FloatUnion>(&(msg_->data[1]),
                                                                                       &propMsgStruct.kd);
        break;

    default:
        LOG(WARN, "Unknowed \"Message Specific Id\"");
        canBusManager_->setWarningFlag();
    }
}
