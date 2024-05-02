#include "Arduino.h"

#include "config_local.hpp"

#include "rover_helpers/helpers.hpp"
#include "rover_can_lib/rover_can_lib.hpp"

#include "rover_can_lib/msgs/cam_control.hpp"
#include "rover_can_lib/msgs/cam_control_a2.hpp"
#include "rover_can_lib/msgs/light_control.hpp"

void canCB(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_);
void parseDeviceIdMsg(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_);
void controlCamera(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_, gpio_num_t camGPIO_);

void setup()
{
    Serial.begin(115200);

    RoverCanLib::CanBusManager canBus(DEVICE_ID, CAN_TX, CAN_RX, canCB, false, (gpio_num_t)LED_BUILTIN);

    canBus.init();

    pinMode(CAM_ENABLE_A2, OUTPUT);
    pinMode(CAM_ENABLE_R1M_1, OUTPUT);
    pinMode(CAM_ENABLE_R1M_2, OUTPUT);
    pinMode(CAM_ENABLE_R1M_3, OUTPUT);
    pinMode(LIGHT_ENABLE, OUTPUT);

    digitalWrite(CAM_ENABLE_A2, HIGH);
    digitalWrite(CAM_ENABLE_R1M_1, HIGH);
    digitalWrite(CAM_ENABLE_R1M_2, HIGH);
    digitalWrite(CAM_ENABLE_R1M_3, HIGH);
    digitalWrite(LIGHT_ENABLE, LOW);

    for (;;)
    {
        canBus.update();
    }
}

void loop() {}

void canCB(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_)
{
    switch ((RoverCanLib::Constant::eDeviceId)msg_->identifier)
    {
    case (RoverCanLib::Constant::eDeviceId::LIGHTS):
    {
        RoverCanLib::Msgs::lightControl lightControlMsg;
        canBusManager_->sendErrorCode(lightControlMsg.parseMsg(msg_));

        if (lightControlMsg.data.enable)
        {
            digitalWrite(LIGHT_ENABLE, HIGH);
        }
        else
        {
            digitalWrite(LIGHT_ENABLE, LOW);
        }

        break;
    }

    case (RoverCanLib::Constant::eDeviceId::CAMERA_A2):
    {
        RoverCanLib::Msgs::camControlA2 camMsg;
        canBusManager_->sendErrorCode(camMsg.parseMsg(msg_));

        if (camMsg.data.enable)
        {
            digitalWrite(CAM_ENABLE_A2, HIGH);
        }
        else
        {
            digitalWrite(CAM_ENABLE_A2, LOW);
        }

        if (camMsg.data.posTilt != 0.0f && camMsg.data.posYaw != 0.0f)
        {
            canBusManager_->sendErrorCode(RoverCanLib::Constant::eInternalErrorCode::WARNING);
            LOG(WARN, "Tilt and yaw not implemented yet")
        }
        
        break;
    }

    case (RoverCanLib::Constant::eDeviceId::CAMERA_R1M_1):
        controlCamera(canBusManager_, msg_, CAM_ENABLE_R1M_1);
        break;

    case (RoverCanLib::Constant::eDeviceId::CAMERA_R1M_2):
        controlCamera(canBusManager_, msg_, CAM_ENABLE_R1M_2);
        break;

    case (RoverCanLib::Constant::eDeviceId::CAMERA_R1M_3):
        controlCamera(canBusManager_, msg_, CAM_ENABLE_R1M_3);
        break;

    default:
        break;
    }
}

void controlCamera(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_, gpio_num_t camGPIO_)
{
    RoverCanLib::Msgs::camControl camMsg;
    canBusManager_->sendErrorCode(camMsg.parseMsg(msg_));

    if (camMsg.data.enable)
    {
        digitalWrite(camGPIO_, HIGH);
    }
    else
    {
        digitalWrite(camGPIO_, LOW);
    }
}
