#include "Arduino.h"

#include "config_local.hpp"

#include "rover_helpers/helpers.hpp"
#include "rover_can_lib/rover_can_lib.hpp"

#include "rover_can_lib/msgs/cam_control.hpp"
#include "rover_can_lib/msgs/cam_control_a2.hpp"

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

    digitalWrite(CAM_ENABLE_A2, LOW);
    digitalWrite(CAM_ENABLE_R1M_1, LOW);
    digitalWrite(CAM_ENABLE_R1M_2, LOW);
    digitalWrite(CAM_ENABLE_R1M_3, LOW);

    for (;;)
    {
        canBus.update();

        if (!canBus.isOk())
        {
            digitalWrite(CAM_ENABLE_A2, LOW);
            digitalWrite(CAM_ENABLE_R1M_1, LOW);
            digitalWrite(CAM_ENABLE_R1M_2, LOW);
            digitalWrite(CAM_ENABLE_R1M_3, LOW);
        }
    }
}

void loop() {}

void canCB(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_)
{
    switch ((RoverCanLib::Constant::eDeviceId)msg_->identifier)
    {
    case (RoverCanLib::Constant::eDeviceId::LIGHTS):
#warning TODO
        break;

    case (RoverCanLib::Constant::eDeviceId::CAMERA_A2):
        if (msg_->data_length_code < 3)
        {
            LOG(WARN, "Ill formed msg, dropping");
            canBusManager_->sendErrorCode(RoverCanLib::Constant::eInternalErrorCode::WARNING);
            return;
        }
        else
        {
            RoverCanLib::Msgs::camControlA2 camMsg;
            RoverCanLib::Constant::eInternalErrorCode errorCode = camMsg.parseMsg(msg_);
            if (errorCode != RoverCanLib::Constant::eInternalErrorCode::OK)
            {
                canBusManager_->sendErrorCode(errorCode);
                LOG(WARN, "Error copying message, dropping...");
                return;
            }

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
        }
        break;

    case (RoverCanLib::Constant::eDeviceId::CAMERA_R1M_1):
        LOG(INFO, "Here");
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
    if (msg_->data_length_code < 3)
    {
        LOG(WARN, "Ill formed msg, dropping");
        canBusManager_->sendErrorCode(RoverCanLib::Constant::eInternalErrorCode::WARNING);
        return;
    }
    else
    {
        RoverCanLib::Msgs::camControl camMsg;
        RoverCanLib::Constant::eInternalErrorCode errorCode;
        errorCode = camMsg.parseMsg(msg_);

        if (errorCode != RoverCanLib::Constant::eInternalErrorCode::OK)
        {
            canBusManager_->sendErrorCode(errorCode);
            LOG(WARN, "Error while parsing msg, dropping...");
            return;
        }

        if (camMsg.data.enable)
        {
            LOG(INFO, "High!");
            digitalWrite(camGPIO_, HIGH);
        }
        else
        {
            LOG(INFO, "Low!");
            digitalWrite(camGPIO_, LOW);
        }
    }
}
