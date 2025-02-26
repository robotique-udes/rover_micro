#include <gtest/gtest.h>
#include "rover_can2/can_device.hpp"

#include "rover_can2/msgs/test_message.hpp"

// =============================================================================
// Helpers
// =============================================================================
namespace TestCanDevice
{
    size_t g_callbackCounter = 0U;

    void CB_Helper(const RoverCan2::Msgs::TestMsg&)
    {
        g_callbackCounter++;
    }
}  // namespace TestCanDevice

// =============================================================================
// Suite
// =============================================================================
TEST(SUITE_ROVER_CAN2_CanDevice, Construction)
{
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanDevice::CB_Helper)> sub0(TestCanDevice::CB_Helper);
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanDevice::CB_Helper)> sub1(TestCanDevice::CB_Helper);

    RoverCan2::CanDevice<2> device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, {sub0, sub1});
}

TEST(SUITE_ROVER_CAN2_CanDevice, Construction_PublishOnly)
{
    RoverCan2::CanDevice<0> device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, {});
}

TEST(SUITE_ROVER_CAN2_CanDevice, MessageParsing)
{
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanDevice::CB_Helper)> sub0(TestCanDevice::CB_Helper);
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanDevice::CB_Helper)> sub1(TestCanDevice::CB_Helper);
    RoverCan2::CanDevice<2> device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, {sub0, sub1});

    uint8_t data[8] = {TO_UNDERLYING(RoverCan2::Constant::eMsgId::TEST_MSG), 0x02, 0x00};
    RoverCan2::CanMsg msg(RoverCan2::Constant::eDeviceId::TEST_DEVICE, data, 3U);

    TestCanDevice::g_callbackCounter = 0UL;

    device.parseMsg(msg);

    ASSERT_TRUE(TestCanDevice::g_callbackCounter == 2);
}
