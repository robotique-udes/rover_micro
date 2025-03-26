#include <gtest/gtest.h>
#include "rover_can2/subscriber.hpp"

#include "rover_can2/msgs/test_msg.hpp"

// =============================================================================
// Helpers
// =============================================================================
class TestHelperClass
{
  public:
    void simulateMsgReception(void)
    {
        uint8_t data[8] = {TO_UNDERLYING(RoverCan2::Constant::eMsgId::TEST_MSG), 0x01, 0x00};
        RoverCan2::CanMsg msg(RoverCan2::Constant::eDeviceId::COMPASS, data, 3U);

        _sub.parseMsg(msg);
    }

    size_t getCallbackCallCnt(void)
    {
        return _callbackCalledCtn;
    }

  private:
    void CB_TestMessage(const RoverCan2::Msgs::TestMsg&)
    {
        _callbackCalledCtn++;
    }

    RoverCan2::SubscriberMember<RoverCan2::Msgs::TestMsg, TestHelperClass> _sub
        = RoverCan2::SubscriberMember<RoverCan2::Msgs::TestMsg, TestHelperClass>(this, &TestHelperClass::CB_TestMessage);

    size_t _callbackCalledCtn = 0;
};
// =============================================================================
// Suite
// =============================================================================
TEST(SUITE_ROVER_CAN2_SubscriberMember, Construction)
{
    TestHelperClass customClass;
}

TEST(SUITE_ROVER_CAN2_SubscriberMember, Callback_On_Last_Elem)
{
    TestHelperClass customClass;

    customClass.simulateMsgReception();
    customClass.simulateMsgReception();

    ASSERT_TRUE(customClass.getCallbackCallCnt() == 2UL);
}

// Base class tests are handled with the SubscriberStandalone
