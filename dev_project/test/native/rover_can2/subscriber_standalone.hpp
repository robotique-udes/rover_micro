#include <gtest/gtest.h>
#include "rover_can2/subscriber.hpp"

#include "rover_can2/msgs/test_msg.hpp"
#include "rover_can2/msgs/test_msg_2.hpp"

// =============================================================================
// Helpers
// =============================================================================
namespace TestSubscriber
{
    bool g_callbackSuccess = false;

    void CB_Helper(const RoverCan2::Msgs::TestMsg&)
    {
        g_callbackSuccess = true;
    }
}  // namespace TestSubscriber
// =============================================================================
// Suite
// =============================================================================
TEST(SUITE_ROVER_CAN2_Subscriber, Construction)
{
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestSubscriber::CB_Helper)> sub(TestSubscriber::CB_Helper);
}

TEST(SUITE_ROVER_CAN2_Subscriber, Callback_On_Last_Elem)
{
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestSubscriber::CB_Helper)> sub(TestSubscriber::CB_Helper);

    uint8_t data[8] = {TO_UNDERLYING(RoverCan2::Constant::eMsgId::TEST_MSG), 0x02, 0x00};
    RoverCan2::CanMsg msg(RoverCan2::Constant::eDeviceId::TEST_DEVICE, data, 3U);

    TestSubscriber::g_callbackSuccess = false;
    RoverCan2::Msgs::Msg::eLoadMsgCode parseCode = sub.parseMsg(msg);

    ASSERT_TRUE(TestSubscriber::g_callbackSuccess == true);
    ASSERT_TRUE(parseCode == RoverCan2::Msgs::Msg::eLoadMsgCode::SUCCESS_COMPLETE);
}

TEST(SUITE_ROVER_CAN2_Subscriber, No_Callback_On_Not_Last_Elem)
{
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestSubscriber::CB_Helper)> sub(TestSubscriber::CB_Helper);

    uint8_t data[8] = {TO_UNDERLYING(RoverCan2::Constant::eMsgId::TEST_MSG), 0x01, 0x00, 0x00, 0x00, 0x00};
    RoverCan2::CanMsg msg(RoverCan2::Constant::eDeviceId::TEST_DEVICE, data, 6U);

    TestSubscriber::g_callbackSuccess = false;
    RoverCan2::Msgs::Msg::eLoadMsgCode parseCode = sub.parseMsg(msg);

    ASSERT_TRUE(TestSubscriber::g_callbackSuccess == false);
    ASSERT_TRUE(parseCode == RoverCan2::Msgs::Msg::eLoadMsgCode::SUCCESS_INCOMPLETE);
}

TEST(SUITE_ROVER_CAN2_Subscriber, Missmatch_On_Not_Concerned_Msg)
{
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestSubscriber::CB_Helper)> sub(TestSubscriber::CB_Helper);

    uint8_t data[8] = {TO_UNDERLYING(RoverCan2::Constant::eMsgId::TEST_MSG_2)};
    RoverCan2::CanMsg msg(RoverCan2::Constant::eDeviceId::TEST_DEVICE, data, 2U);

    TestSubscriber::g_callbackSuccess = false;
    RoverCan2::Msgs::Msg::eLoadMsgCode parseCode = sub.parseMsg(msg);

    ASSERT_TRUE(TestSubscriber::g_callbackSuccess == false);
    ASSERT_TRUE(parseCode == RoverCan2::Msgs::Msg::eLoadMsgCode::NOT_CONCERNED);
}

TEST(SUITE_ROVER_CAN2_Subscriber, Missmatch_On_Invalid_Msg)
{
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestSubscriber::CB_Helper)> sub(TestSubscriber::CB_Helper);

    uint8_t data[10] = {TO_UNDERLYING(RoverCan2::Constant::eMsgId::TEST_MSG)};
    RoverCan2::CanMsg msg(RoverCan2::Constant::eDeviceId::TEST_DEVICE, data, sizeof(data));

    TestSubscriber::g_callbackSuccess = false;
    RoverCan2::Msgs::Msg::eLoadMsgCode parseCode = sub.parseMsg(msg);

    ASSERT_TRUE(TestSubscriber::g_callbackSuccess == false);
    ASSERT_TRUE(parseCode == RoverCan2::Msgs::Msg::eLoadMsgCode::ERROR_INVALID_MSG);
}

TEST(SUITE_ROVER_CAN2_Subscriber, Missmatch_On_Invalid_Length)
{
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestSubscriber::CB_Helper)> sub(TestSubscriber::CB_Helper);

    uint8_t data[8] = {TO_UNDERLYING(RoverCan2::Constant::eMsgId::TEST_MSG)};
    RoverCan2::CanMsg msg(RoverCan2::Constant::eDeviceId::TEST_DEVICE, data, 2U);

    TestSubscriber::g_callbackSuccess = false;
    RoverCan2::Msgs::Msg::eLoadMsgCode parseCode = sub.parseMsg(msg);

    ASSERT_TRUE(TestSubscriber::g_callbackSuccess == false);
    ASSERT_TRUE(parseCode == RoverCan2::Msgs::Msg::eLoadMsgCode::ERROR_MISSMATCH);
}

TEST(SUITE_ROVER_CAN2_Subscriber, Missmatch_On_Invalid_MsgContentID)
{
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestSubscriber::CB_Helper)> sub(TestSubscriber::CB_Helper);

    uint8_t data[8] = {TO_UNDERLYING(RoverCan2::Constant::eMsgId::TEST_MSG), 0x05};
    RoverCan2::CanMsg msg(RoverCan2::Constant::eDeviceId::TEST_DEVICE, data, 2U);

    TestSubscriber::g_callbackSuccess = false;
    RoverCan2::Msgs::Msg::eLoadMsgCode parseCode = sub.parseMsg(msg);

    ASSERT_TRUE(TestSubscriber::g_callbackSuccess == false);
    ASSERT_TRUE(parseCode == RoverCan2::Msgs::Msg::eLoadMsgCode::ERROR_MISSMATCH);
}
