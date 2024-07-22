#include <Arduino.h>

#include "rover_can_lib/can_bus_manager.hpp"
#include "rover_can_lib/msgs/compass.hpp"

#include "config_local.hpp"
#include "rover_helpers/helpers.hpp"
#include "rover_helpers/log.hpp"
#include "sensors/WT901.hpp"

void noActions(RoverCanLib::CanBusManager *dontUse0_, const twai_message_t *dontUse1_);
void parseAnglePacket();
void calibrate();

void setup() 
{
    delay(2500);
    Serial.begin(115200);

    WT901 compass(RX1_PIN, TX1_PIN);
    RoverCanLib::CanBusManager canBus(DEVICE_ID, CAN_TX, CAN_RX, noActions, true);

    compass.init();
    canBus.init();

    RoverCanLib::Msgs::Compass compassMsg;
    RoverHelpers::Timer<unsigned long, millis> timerFeedback(500);

    LOG(INFO, "Init done, starting loop!");

    for(;;)
    {
        compass.updateOrientation();
        canBus.update();
        
        if (timerFeedback.isDone())
        {
            compassMsg.data.heading = compass.getYaw();
            compassMsg.data.pitch = compass.getPitch();

            canBus.sendMsg(&compassMsg);
        }
    }
}


void loop() {}

void noActions(RoverCanLib::CanBusManager *dontUse0_, const twai_message_t *dontUse1_)
{
    REMOVE_UNUSED(&dontUse0_);
    REMOVE_UNUSED(dontUse1_);

    return;
} 