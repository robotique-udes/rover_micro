#include "rover_can2/can_device.hpp"
#include "rover_can2/can_driver.hpp"
#include "rover_can2/msgs/motor_cmd.hpp"

#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/chrono.hpp"

#include "rover_lib2/helpers/static_array.hpp"

#include <Arduino.h>
#include <functional>

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

void CB_test(const RoverCan2::Msgs::MotorCmd& msg);

void setup()
{
    Serial.begin(115200UL);
    delay(1000);

    RoverCan2::CanDriver driver(GPIO_NUM_47, GPIO_NUM_48);
    driver.init();

    RoverCan2::Subscriber<RoverCan2::Msgs::MotorCmd, std::function<void(const RoverCan2::Msgs::MotorCmd&)>> msgSub(
        [](const auto& msg)
        {
            CB_test(msg);
        });

    for (EVER)
    {
        driver.update();

        if (auto canMsg = driver.getMsg())
        {
            msgSub.parseMsg(canMsg.value());
        }
    }
}

void loop() {}

void CB_test(const RoverCan2::Msgs::MotorCmd& msg)
{
    LOG_INFO(Logger::Nodes::Main,
              "Here! Damn ça marche...? msg.cmd: %f, msg.closeLoop: %d",
              msg.data().cmd,
              msg.data().closeLoop);
}
