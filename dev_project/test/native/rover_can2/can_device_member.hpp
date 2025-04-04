#include <gtest/gtest.h>

#include "rover_can2/can_device.hpp"
#include "rover_can2/subscriber.hpp"

#include "rover_can2/msgs/test_msg.hpp"

// =============================================================================
// Helpers
// =============================================================================
namespace TestCanDeviceMember
{
    bool testVariable = false;

    class CustomDevice : public RoverCan2::CanDevice<RoverCan2::SubscriberMember<RoverCan2::Msgs::TestMsg, CustomDevice>>
    {
      public:
        CustomDevice():
            RoverCan2::CanDevice<RoverCan2::SubscriberMember<RoverCan2::Msgs::TestMsg, CustomDevice>>(
                RoverCan2::Constant::eDeviceId::TEST_DEVICE,
                RoverCan2::SubscriberMember<RoverCan2::Msgs::TestMsg, CustomDevice>(*this, &CustomDevice::CB_testMsg))
        {
        }

      private:
        void CB_testMsg(const RoverCan2::Msgs::TestMsg& /*msg_*/)
        {
            testVariable = true;
        }
    };

}  // namespace TestCanDeviceMember

// =============================================================================
// Suite
// =============================================================================

TEST(SUITE_ROVER_CAN2_CanDeviceMember, Construction)
{
    TestCanDeviceMember::CustomDevice device;
}

TEST(SUITE_ROVER_CAN2_CanDeviceMember, Callback)
{
    TestCanDeviceMember::CustomDevice device;

    RoverCan2::Msgs::TestMsg msg;
    RoverCan2::CanMsg canMsg = msg.getCanMsg(msg.getMsgContentCount() - 1UL).value();
    canMsg.setCanID(RoverCan2::Constant::eDeviceId::TEST_DEVICE);

    GTEST_ASSERT_TRUE(TestCanDeviceMember::testVariable == false);
    device.parseMsg(canMsg);
    GTEST_ASSERT_TRUE(TestCanDeviceMember::testVariable == true);
}
