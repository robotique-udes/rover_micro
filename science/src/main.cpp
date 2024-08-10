#include <Arduino.h>

#include "config_local.hpp"
#include "rover_helpers/helpers.hpp"
#include "rover_can_lib/rover_can_lib.hpp"
#include "actuators/motor_drivers/DRV8251A.hpp"
#include "sensors/limit_switch.hpp"
#include "rover_can_lib/msgs/science.hpp"
#include "actuators/servo.h"

void canCB(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_);
void parseDeviceIdMsg(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_);
Servo scienceServo(SERVO_0, 500.0f, 2650.0f, 840.0f);

float g_servoPos = 840.0f;

RoverCanLib::Msgs::Science msgScience;

#define DOWN 0
#define UP 1

void setup()
{
    delay(2500);
    Serial.begin(115200);

    LimitSwitch switchUp(LimitSwitch::eLimitSwitchMode::PullDown, PB_UP);
    LimitSwitch switchDown(LimitSwitch::eLimitSwitchMode::PullDown, PB_DOWN);
    DRV8251A actuator(LIN_IN_1, LIN_IN_2);

    RoverCanLib::CanBusManager canBus(DEVICE_ID, PIN_CAN_TX, PIN_CAN_RX, canCB, true);
    canBus.init();

    switchUp.init();
    switchDown.init();
    actuator.init();
    scienceServo.init();

    pinMode(FAN_A_PWM, OUTPUT);
    pinMode(GRINDER_PWM, OUTPUT);
    pinMode(PB_GRINDER, INPUT);

    RoverHelpers::Timer<unsigned long, millis> timerSetCmd(10);

    float speedCmd = 0.0f;

    LOG(INFO, "Init done, starting loop!");

    for (EVER)
    {
        canBus.update();

        if (timerSetCmd.isDone())
        {
            if (canBus.isOk() && msgScience.data.cmd == 0 && !switchDown)
            {
                if (DEVICE_ID == (uint16_t)RoverCanLib::Constant::eDeviceId::SCIENCE)
                {
                    speedCmd = 100.0f;
                }
            }
            else if ()
            {
                speedCmd = 0.0f;
            }

            if (canBus.isOk() && msgScience.data.dig)
            {
                if (DEVICE_ID == (uint16_t)RoverCanLib::Constant::eDeviceId::SCIENCE)
                {
                    digitalWrite(GRINDER_PWM, HIGH);
                    digitalWrite(FAN_A_PWM, HIGH);
                }
            }
            else
            {
                digitalWrite(GRINDER_PWM, LOW);
                digitalWrite(FAN_A_PWM, LOW);
            }

            if (canBus.isOk() &&msgScience.data.current_sample = 0)
            {
                if (DEVICE_ID == (uint16_t)RoverCanLib::Constant::eDeviceId::SCIENCE)
                {
                    g_servoPos = MAP(15.0f, 0.0f, 360.0f, 500.0f, 2650.0f);
                }
            }
            else
            {
                g_servoPos = MAP(15.0f, 0.0f, 360.0f, 500.0f, 2650.0f);
            }

            if (canBus.isOk() &&msgScience.data.current_sample = 1)
            {
                if (DEVICE_ID == (uint16_t)RoverCanLib::Constant::eDeviceId::SCIENCE)
                {
                    g_servoPos = MAP(50.0f, 0.0f, 360.0f, 500.0f, 2650.0f);
                }
            }
            else
            {
                g_servoPos = MAP(15.0f, 0.0f, 360.0f, 500.0f, 2650.0f);
            }

            if (canBus.isOk() &&msgScience.data.current_sample = 2)
            {
                if (DEVICE_ID == (uint16_t)RoverCanLib::Constant::eDeviceId::SCIENCE)
                {
                    g_servoPos = MAP(450.0f, 0.0f, 360.0f, 500.0f, 2650.0f);
                }
            }
            else
            {
                g_servoPos = MAP(15.0f, 0.0f, 360.0f, 500.0f, 2650.0f);
            }

            actuator.setCmd(speedCmd);
            scienceServo.writeMicroseconds(g_servoPos);
        }

        if (timerSetCmd.isDone())
        {
            if (canBus.isOk() && msgScience.data.cmd == 0)
            {
                LOG(INFO, "LINEAR ACTUATOR UP");
            }
            else if (canBus.isOk() && msgScience.data.cmd == 1)
            {
                LOG(INFO, "LINEAR ACTUATOR DOWN");
            }

            if (canBus.isOk() && msgScience.data.dig == true)
            {
                LOG(INFO, "DIG MOTHERFUCKER");
            }
            else
            {
                LOG(INFO, "NO DIGGING");
            }
        }
    }
}

void canCB(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_)
{
    // switch (msg_->identifier)
    // {
    // case (DEVICE_ID)
    //     if(msg_)
    // }
}

void loop() {}