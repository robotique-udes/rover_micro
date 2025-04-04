#include <gtest/gtest.h>

#include "rover_can2/drivers/can_driver_mock.hpp"
#include "rover_can2/publisher.hpp"
#include "rover_can2/msgs/test_msg.hpp"
#include "rover_can2/can_device.hpp"
#include "rover_can2/can_manager.hpp"

// =============================================================================
// Helpers
// =============================================================================
namespace TestPublisher
{

}  // namespace TestPublisher

// =============================================================================
// Suite
// =============================================================================

TEST(SUITE_NAME_Publisher, Construction)
{
    RoverCan2::Publisher<RoverCan2::Msgs::TestMsg> pub;
}

TEST(SUITE_NAME_Publisher, SimplePublish)
{
    RoverCan2::Publisher<RoverCan2::Msgs::TestMsg> pub;

    RoverCan2::Drivers::CanDriverMock driver;
    RoverCan2::CanDevice device(RoverCan2::Constant::eDeviceId::TEST_DEVICE);
    RoverCan2::CanManager manager(driver, device);
    manager.init();

    RoverCan2::Msgs::TestMsg msg;
    msg.data().cmd = 69.0F;
    msg.data().closeLoop = true;
    pub.queueMsg(msg);

    manager.update();
    pub.sendQueuedMsgs(RoverCan2::Constant::eDeviceId::TEST_DEVICE, manager);

    GTEST_ASSERT_TRUE(driver.msgSentBuffer.size() == TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::eLAST));
}

TEST(SUITE_NAME_Publisher, MultiplePublish)
{
    RoverCan2::Publisher<RoverCan2::Msgs::TestMsg> pub;

    RoverCan2::Drivers::CanDriverMock driver;
    RoverCan2::CanDevice device(RoverCan2::Constant::eDeviceId::TEST_DEVICE);
    RoverCan2::CanManager manager(driver, device);
    manager.init();

    RoverCan2::Msgs::TestMsg msg;
    msg.data().cmd = 69.0F;
    msg.data().closeLoop = true;
    pub.queueMsg(msg);
    pub.queueMsg(msg);
    pub.queueMsg(msg);

    manager.update();
    pub.sendQueuedMsgs(RoverCan2::Constant::eDeviceId::TEST_DEVICE, manager);

    GTEST_ASSERT_TRUE(driver.msgSentBuffer.size() == 3UL * TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::eLAST));
}
