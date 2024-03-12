#include "Arduino.h"

#include "config_local.hpp"

#include "helpers/helpers.hpp"
#include "rover_can_lib/rover_can_lib.hpp"
#include "rover_can_lib/msgs/propulsion_motor.hpp"

#include "actuators/talon_srx.hpp"

#define PIN_PWM 23
#define PIN_GND 22

void canCB(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_);
void parseDeviceIdMsg(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_);

RoverCanLib::Msgs::PropulsionMotor msgProp;

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

        if (canBus.isOk() && msgProp.data.enable)
        {
            motorDrive.writeMicroseconds(MAP(msgProp.data.targetSpeed, -100.0f, 100.0f, 1000.0f, 2000.0f));
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
        if (msg_->data_length_code < 3)
        {
            LOG(WARN, "Ill formed msg, dropping");
            canBusManager_->sendErrorCode(RoverCanLib::Constant::eInternalErrorCode::WARNING);
        }
        else
        {
            canBusManager_->resetWatchDog();
            canBusManager_->sendErrorCode(msgProp.parseMsg(msg_));
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
