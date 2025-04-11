#include <Arduino.h>

#include "rover_can2/msgs/test_msg.hpp"
#include "rover_can2/rover_can.hpp"

#include "rover_lib2/helpers/macros.hpp"
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/one_shot_timer.hpp"
#include "rover_lib2/helpers/time.hpp"

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_47;
constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_48;

class ExampleCanDevice : public RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::TestMsg, ExampleCanDevice>,
                                                  RoverCan2::Publisher<RoverCan2::Msgs::ErrorState>>
{
  public:
    ExampleCanDevice():
        RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::TestMsg, ExampleCanDevice>,
                          RoverCan2::Publisher<RoverCan2::Msgs::ErrorState>>(
            RoverCan2::Constant::eDeviceId::TEST_DEVICE,
            RoverCan2::SubscriberMember(*this, &ExampleCanDevice::CB_testMsg),
            RoverCan2::Publisher<RoverCan2::Msgs::ErrorState>())

    {
    }

  private:
    void CB_testMsg(const RoverCan2::Msgs::TestMsg& msg_)
    {
        (void)msg_;

        LOG_INFO(Logger::Nodes::Main,
                 "New msg received: msg_.cmd: %f, msg_.closeLoop: %d",
                 msg_.getData().cmd,
                 msg_.getData().closeLoop);

        RoverCan2::Msgs::ErrorState msg;
        msg.data().error = true;
        this->sendMsg<RoverCan2::Msgs::ErrorState>(msg);
    }
};

void setup(void)
{
    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif

    RoverCan2::Drivers::DriverESP32 canDriver(PIN_CAN_RX, PIN_CAN_TX);
    ExampleCanDevice device;
    RoverCan2::Manager canManager(canDriver, device);
    canManager.init();

    LOG_INFO(Logger::Nodes::Main, "Init done, starting loop!");
    for (EVER)
    {
        canManager.update();
    }
}

void loop() {}
