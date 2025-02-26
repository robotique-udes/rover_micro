#include "rover_can2/can_driver.hpp"
#include "rover_can2/msgs/motor_cmd.hpp"
#include "rover_can2/subscriber.hpp"

#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/chrono.hpp"

#include "rover_lib2/helpers/static_array.hpp"

#include <Arduino.h>
#include <functional>

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

void CB_test(const RoverCan2::Msgs::MotorCmd& msg);

class CanDevice : public RoverObject
{
  public:
    CanDevice(RoverCan2::Constant::eDeviceId id_):
        _id(id_)
    {
    }

  private:
    RoverCan2::Constant::eDeviceId _id;
    // StaticArray<RoverCan2::Constant::eMsgId> _subscribedMsgId;
};

class MyDevice
{
  public:
    MyDevice():
        _subMotorCmd(this, &MyDevice::CB_motorCmd)
    {
    }

    void parseMsg(const RoverCan2::CanMsg& canMsg_)
    {
        _subMotorCmd.parseMsg(canMsg_);
    }

  private:
    void CB_motorCmd(const RoverCan2::Msgs::MotorCmd& msg_)
    {
        LOG_INFO(Logger::Nodes::Main, "Working!");
    }

    RoverCan2::SubscriberMember<RoverCan2::Msgs::MotorCmd, MyDevice> _subMotorCmd;
};

void setup()
{
    Serial.begin(115200UL);
    delay(1000);

    RoverCan2::CanDriver driver(GPIO_NUM_47, GPIO_NUM_48);
    driver.init();

    MyDevice device;

    for (EVER)
    {
        driver.update();

        if (auto canMsg = driver.getMsg())
        {
            device.parseMsg(canMsg.value());
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
