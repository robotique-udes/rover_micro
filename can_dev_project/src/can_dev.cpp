#include "Arduino.h"

#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/macros.hpp"
// #include "rover_helpers/helpers.hpp"

// #include "rover_can2/can_driver.hpp"
#include "rover_can2/can_device.hpp"
#include "rover_can2/can_manager.hpp"

#include <functional>

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

void CB_BMS(const float& /*msg_*/, int);
void CB_cameraPan(const uint8_t& /*msg_*/, float a);
void CB_compass(const std::array<uint8_t, 8UL>& /*msg_*/, const char* a);

void setup()
{
    Serial.begin(115200UL);
    delay(1000);

    int a = 10;
    float b = 20;
    const char* c = "Hello";

    RoverCan2::CanDevice<RoverCan2::Constant::eDeviceId::BMS, float, decltype(a)> BMS(CB_BMS, a);
    RoverCan2::CanDevice<RoverCan2::Constant::eDeviceId::CAMERA_PAN, uint8_t, decltype(b)> camPan(CB_cameraPan, b);
    RoverCan2::CanDevice<RoverCan2::Constant::eDeviceId::COMPASS, std::array<uint8_t, 8UL>, decltype(c)> compass(CB_compass, c);

    RoverCan2::CanManager canManager(GPIO_NUM_47, GPIO_NUM_48, BMS, camPan, compass);
    canManager.init();

    for (EVER)
    {
        canManager.update();
    }
}

void loop() {}

void CB_BMS(const float& /*msg_*/, int a)
{
    LOG::INFO(Logger::Nodes::Main, "Here %i", a);
}

void CB_cameraPan(const uint8_t& /*msg_*/, float a)
{
    LOG::INFO(Logger::Nodes::Main, "Here %f", a);
}

void CB_compass(const std::array<uint8_t, 8UL>& /*msg_*/, const char* a)
{
    LOG::INFO(Logger::Nodes::Main, "Here %s", a);
}
