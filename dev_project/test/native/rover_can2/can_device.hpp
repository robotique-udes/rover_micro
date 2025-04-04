#include <gtest/gtest.h>

#include "rover_can2/can_device.hpp"
#include "rover_can2/msgs/test_msg.hpp"

// #error REDO WITHOUT DRIVER IMPL

// =============================================================================
// Helpers
// =============================================================================
namespace TestCanDevice
{
    bool testValue = false;
    size_t g_callbackCounter = 0UL;
    void CB_Helper(const RoverCan2::Msgs::TestMsg& msg_)
    {
        testValue = msg_.getData().closeLoop;
        g_callbackCounter++;
    }

    void CB_Helper2(const RoverCan2::Msgs::TestMsg&)
    {
        g_callbackCounter += 2UL;
    }
}  // namespace TestCanDevice

// =============================================================================
// Suite
// =============================================================================
TEST(SUITE_ROVER_CAN2_CanDevice, Construction)
{
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanDevice::CB_Helper)> sub0(TestCanDevice::CB_Helper);
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanDevice::CB_Helper)> sub1(TestCanDevice::CB_Helper);
    RoverCan2::CanDevice device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, sub1, sub0);
}

TEST(SUITE_ROVER_CAN2_CanDevice, IdReporting)
{
    RoverCan2::CanDevice device(RoverCan2::Constant::eDeviceId::TEST_DEVICE);
    GTEST_ASSERT_TRUE(device.getCanId() == RoverCan2::Constant::eDeviceId::TEST_DEVICE);
}

TEST(SUITE_ROVER_CAN2_CanDevice, ValidMsgRecv)
{
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanDevice::CB_Helper)> sub1(TestCanDevice::CB_Helper);
    RoverCan2::CanDevice device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, sub1);

    // Manually building a CAN test message to simulate the reception of a message
    // This message is the last MSG_CONTENT_ID of the "TEST_MESSAGE" which should trigger a callback if a subscriber of this type
    // exist
    std::array<uint8_t, sizeof(bool) + TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA)> data
        = {/*MsgID=TEST_MSG*/ TO_UNDERLYING(RoverCan2::Constant::eMsgId::TEST_MSG),
           /*MSG_CONTENT_ID=CMD*/ TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CLOSE_LOOP),
           /* DATA 0 = true */ 0x01};

    RoverCan2::CanMsg msg(RoverCan2::Constant::eDeviceId::TEST_DEVICE, data.data(), data.size());
    // Simulate the reception of a msg
    TestCanDevice::g_callbackCounter = 0UL;
    GTEST_ASSERT_TRUE(device.parseMsg(msg) == true);

    // The callback should have been called
    GTEST_ASSERT_TRUE(TestCanDevice::g_callbackCounter == 1UL);
}

TEST(SUITE_ROVER_CAN2_CanDevice, ValidMsgRecvMultipleSubs)
{
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanDevice::CB_Helper)> sub0(TestCanDevice::CB_Helper);
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanDevice::CB_Helper2)> sub1(
        TestCanDevice::CB_Helper2);
    RoverCan2::CanDevice device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, sub0, sub1);

    // Manually building a CAN test message simulate the reception of a message
    // This message is the last MSG_CONTENT_ID of the "TEST_MESSAGE" which should trigger a callback if a subscriber of this type
    // exist
    std::array<uint8_t, sizeof(bool) + TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA)> data
        = {/*MsgID=TEST_MSG*/ TO_UNDERLYING(RoverCan2::Constant::eMsgId::TEST_MSG),
           /*MSG_CONTENT_ID=CMD*/ TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CLOSE_LOOP),
           /* DATA 0 = true */ 0x01};

    RoverCan2::CanMsg msg(RoverCan2::Constant::eDeviceId::TEST_DEVICE, data.data(), data.size());
    // Simulate the reception of a msg
    TestCanDevice::g_callbackCounter = 0UL;
    GTEST_ASSERT_TRUE(device.parseMsg(msg) == true);

    // The callback should have been called two times, one from each subs
    GTEST_ASSERT_TRUE(TestCanDevice::g_callbackCounter == 3UL);
}

TEST(SUITE_ROVER_CAN2_CanDevice, NotConcernedMsgRecv)
{
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanDevice::CB_Helper)> sub1(TestCanDevice::CB_Helper);
    // RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanDevice::CB_Helper)>
    // sub1(TestCanDevice::CB_Helper);
    RoverCan2::CanDevice device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, sub1);

    // Manually building a CAN test message and adding it to the buffer to simulate the reception of a message by the driver
    // This message is the last MSG_CONTENT_ID of the "TEST_MESSAGE" which should trigger a callback if a subscriber of this type
    // exist
    std::array<uint8_t, sizeof(bool) + TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA)> data
        = {/*MsgID=TEST_MSG*/ TO_UNDERLYING(RoverCan2::Constant::eMsgId::TEST_MSG),
           /*MSG_CONTENT_ID=CMD*/ TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CLOSE_LOOP),
           /* DATA 0 = true */ 0x01};

    RoverCan2::CanMsg msg(RoverCan2::Constant::eDeviceId::COMPASS, data.data(), data.size());
    // Simulate the reception of a msg
    TestCanDevice::g_callbackCounter = 0UL;
    GTEST_ASSERT_TRUE(device.parseMsg(msg) == true);

    // The wanted callback should not trigger because the id doesn't match
    GTEST_ASSERT_TRUE(TestCanDevice::g_callbackCounter == 0);
}

TEST(SUITE_ROVER_CAN2_CanDevice, InvalidMsg)
{
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanDevice::CB_Helper)> sub1(TestCanDevice::CB_Helper);
    RoverCan2::CanDevice device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, sub1);

    // Manually building a CAN test message and adding it to the buffer to simulate the reception of a message by the driver
    // The lenght of the message is invalid so no callback should be called and the "parseMsg(msg)" should return false
    std::array<uint8_t, 10U + TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA)> data
        = {/*MsgID=TEST_MSG*/ TO_UNDERLYING(RoverCan2::Constant::eMsgId::TEST_MSG),
           /*MSG_CONTENT_ID=CMD*/ TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CLOSE_LOOP),
           /* DATA 0 = true */ 0x01};

    RoverCan2::CanMsg msg(RoverCan2::Constant::eDeviceId::TEST_DEVICE, data.data(), data.size());
    // Simulate the reception of a msg
    TestCanDevice::g_callbackCounter = 0UL;
    GTEST_ASSERT_TRUE(device.parseMsg(msg) == false);

    GTEST_ASSERT_TRUE(TestCanDevice::g_callbackCounter == 0);
}

TEST(SUITE_ROVER_CAN2_CanDevice, ValidMsgRecv_ValidData)
{
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanDevice::CB_Helper)> sub1(TestCanDevice::CB_Helper);
    RoverCan2::CanDevice device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, sub1);

    // Manually building a CAN test message to simulate the reception of a message
    // This message is the last MSG_CONTENT_ID of the "TEST_MESSAGE" which should trigger a callback if a subscriber of this type
    // exist
    std::array<uint8_t, sizeof(bool) + TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA)> data
        = {/*MsgID=TEST_MSG*/ TO_UNDERLYING(RoverCan2::Constant::eMsgId::TEST_MSG),
           /*MSG_CONTENT_ID=CMD*/ TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CLOSE_LOOP),
           /* DATA 0 = true */ 0x01};

    RoverCan2::CanMsg msg(RoverCan2::Constant::eDeviceId::TEST_DEVICE, data.data(), data.size());
    // Simulate the reception of a msg
    TestCanDevice::testValue = false;
    GTEST_ASSERT_TRUE(TestCanDevice::testValue == false);
    GTEST_ASSERT_TRUE(device.parseMsg(msg) == true);
    GTEST_ASSERT_TRUE(TestCanDevice::testValue == true);
}
