#include "Arduino.h"

#include "config_local.hpp"

#include "rover_helpers/helpers.hpp"
#include "actuators/servo.h"
#include "rover_can_lib/rover_can_lib.hpp"

#include "rover_can_lib/msgs/cam_control.hpp"
#include "rover_can_lib/msgs/cam_pan.hpp"
#include "rover_can_lib/msgs/light_control.hpp"

void canCB(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_);
void parseDeviceIdMsg(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_);
void controlCamera(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_, gpio_num_t camGPIO_);

float g_servoPos = 840.0f;

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

    Servo panoCamera(CAM_PIN_PWM, 500.0f, 2650.0f, 840.0f);
    panoCamera.init();

    for (;;)
    {
        panoCamera.writeMicroseconds(g_servoPos);
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

    case (RoverCanLib::Constant::eDeviceId::CAMERA_PAN):
    {
        RoverCanLib::Msgs::CamPan msg;
        canBusManager_->sendErrorCode(msg.parseMsg(msg_));
        g_servoPos = MAP(msg.data.servoPosition, 0.0f, 360.0f, 500.0f, 2650.0f);
        LOG(INFO, "g_servoPos: %f", g_servoPos);
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
