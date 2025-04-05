#include "Arduino.h"

#include "config_local.hpp"

#include "rover_can_lib/can_bus_manager.hpp"
#include "rover_can_lib/msgs/arm_cmd.hpp"
#include "rover_can_lib/msgs/arm_status.hpp"

#include "SPI.h"
#include "actuators/motor_drivers/IFX007T.hpp"
#include "rover_helpers/PID.hpp"
#include "rover_helpers/helpers.hpp"
#include "sensors/encoders/CUI_AMT222.hpp"
#include "sensors/limit_switch.hpp"
#include "systems/joint/dc_revolute_joint.hpp"

void CB_Can(RoverCanLib::CanBusManager* canManager_, const twai_message_t* msgPtr_);

float g_goalPosJ2 = 0.0f;

void setup()
{
    Serial.begin(115200);
    LOG(WARN, "Init done starting!");

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, GPIO_NUM_NC);
    CUI_AMT222 encoder(&SPI, PIN_SPI_CS_EN_SHAFT, true, Encoder::eEncoderType::ABSOLUTE_SINGLE_TURN);
    encoder.init();

    for (;;)
    {
        encoder.update();
        LOG(INFO, "%f", encoder.getPosition(true));
    }
}

void loop() {}

void CB_Can(RoverCanLib::CanBusManager* canManager_, const twai_message_t* msgPtr_)
{
    if (msgPtr_->identifier == (uint32_t)DEVICE_ID)
    {
        canManager_->resetWatchDog();
        RoverCanLib::Msgs::armCmd armMsg;
        canManager_->sendErrorCode(armMsg.parseMsg(msgPtr_));

        g_goalPosJ2 = armMsg.data.targetSpeed;
    }
}
