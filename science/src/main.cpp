#include <Arduino.h>

#include "config_local.hpp"
#include "rover_helpers/helpers.hpp"
#include "rover_can_lib/rover_can_lib.hpp"
#include "actuators/motor_drivers/DRV8251A.hpp"
#include "sensors/limit_switch.hpp"
#include "rover_can_lib/msgs/science.hpp"

void canCB(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_);
void parseDeviceIdMsg(RoverCanLib::CanBusManager *canBusManager_, const twai_message_t *msg_);

RoverCanLib::Msgs::Science msgScience;

void setup()
{
    delay(2500);
    Serial.begin(115200);

    LimitSwitch switchGrinder(LimitSwitch::eLimitSwitchMode::PullDown, PB_GRINDER);
    DRV8251A actuator(LIN_IN_1, LIN_IN_2);

    RoverCanLib::CanBusManager canBus(DEVICE_ID, PIN_CAN_TX, PIN_CAN_RX, canCB, true);
    canBus.init();

    switchGrinder.init();
    actuator.init();

    pinMode(FAN_A_PWM, OUTPUT);
    pinMode(GRINDER_PWM, OUTPUT);
    pinMode(PB_GRINDER, INPUT);

    RoverHelpers::Timer<unsigned long, millis> timerSetCmd(10);
    
    LOG(INFO, "Init done, starting loop!");

    for (EVER)
    {
        canBus.update();

        if (timerSetCmd.isDone())
        {
            if(canBus.isOk() && msgScience.data.cmd == 0)
            {
                LOG(INFO, "LINEAR ACTUATOR UP");
            }
            else if(canBus.isOk() && msgScience.data.cmd == 1)
            {
                LOG(INFO, "LINEAR ACTUATOR DOWN");
            }

            if(canBus.isOk() && msgScience.data.dig == true)
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

}

void loop() {}