#include <gtest/gtest.h>

#include "rover_can2/can_manager.hpp"

#include "rover_can2/msgs/test_msg.hpp"

// =============================================================================
// Helpers
// =============================================================================
namespace TestCanManager
{
#include "rover_can2/drivers/can_driver_mock.hpp"

    size_t g_callbackCounter = 0UL;
    void CB_Helper(const RoverCan2::Msgs::TestMsg&)
    {
        g_callbackCounter++;
    }

    void CB_Helper2(const RoverCan2::Msgs::TestMsg&)
    {
        g_callbackCounter += 2UL;
    }
}  // namespace TestCanManager

// =============================================================================
// Suite
// =============================================================================

TEST(SUITE_NAME_CanManager, Construction)
{
    TestCanManager::CanDriverMock canDriver;
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanManager::CB_Helper)> sub0(
        TestCanManager::CB_Helper);
    RoverCan2::CanDevice device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, sub0);

    RoverCan2::CanManager canManager(canDriver, device);
}

/**
 * @brief Makes sure the init class calls the driver's init
 *
 */
TEST(SUITE_NAME_CanManager, Init)
{
    TestCanManager::CanDriverMock canDriver;
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanManager::CB_Helper)> sub0(
        TestCanManager::CB_Helper);
    RoverCan2::CanDevice device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, sub0);

    RoverCan2::CanManager canManager(canDriver, device);
    canManager.init();

    GTEST_ASSERT_TRUE(canDriver.isInited == true);
}

/**
 * @brief Makes sure the init class calls the driver's update
 *
 */
TEST(SUITE_NAME_CanManager, Update)
{
    TestCanManager::CanDriverMock canDriver;
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanManager::CB_Helper)> sub0(
        TestCanManager::CB_Helper);
    RoverCan2::CanDevice device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, sub0);

    RoverCan2::CanManager canManager(canDriver, device);
    canManager.init();
    canManager.update();

    GTEST_ASSERT_TRUE(canDriver.hasUpdated == true);
}

TEST(SUITE_NAME_CanManager, MsgRecvHandling)
{
    TestCanManager::CanDriverMock canDriver;
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanManager::CB_Helper)> sub0(
        TestCanManager::CB_Helper);
    RoverCan2::CanDevice device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, sub0);

    TestCanManager::g_callbackCounter = 0U;
    RoverCan2::CanManager canManager(canDriver, device);
    canManager.init();
    canManager.update();

    // Creating msg and putting it in the driver's buffer to simulate msg reception
    std::array<uint8_t, sizeof(bool) + TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA)> data
        = {TO_UNDERLYING(RoverCan2::Constant::eMsgId::TEST_MSG),
           TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CLOSE_LOOP),
           0x01};
    RoverCan2::CanMsg msg(RoverCan2::Constant::eDeviceId::TEST_DEVICE, data.data(), data.size());
    canDriver.newMsgsBuffer.addValue(msg);

    GTEST_ASSERT_TRUE(TestCanManager::g_callbackCounter == 0);

    canManager.update();

    // There's one new msg from one of it's device sub so 1 callback should have triggered
    GTEST_ASSERT_TRUE(TestCanManager::g_callbackCounter == 1);
}

TEST(SUITE_NAME_CanManager, MsgRecvHandlingInfiniteLoop)
{
    TestCanManager::CanDriverMock canDriver;
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanManager::CB_Helper)> sub0(
        TestCanManager::CB_Helper);
    RoverCan2::CanDevice device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, sub0);

    TestCanManager::g_callbackCounter = 0U;
    RoverCan2::CanManager canManager(canDriver, device);
    canManager.init();
    canManager.update();

    // Creating msgs and putting them in the driver's buffer to simulate multiple msg reception
    std::array<uint8_t, sizeof(bool) + TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA)> data
        = {TO_UNDERLYING(RoverCan2::Constant::eMsgId::TEST_MSG),
           TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CLOSE_LOOP),
           0x01};
    RoverCan2::CanMsg msg(RoverCan2::Constant::eDeviceId::TEST_DEVICE, data.data(), data.size());

    for (size_t i = 0; i < 1'000; i++)
    {
        canDriver.newMsgsBuffer.addValue(msg);
    }

    GTEST_ASSERT_TRUE(TestCanManager::g_callbackCounter == 0);

    canManager.update();

    // There's multiple new msgs from one of it's device sub so 5 callback should
    // have triggered has this is the hardcoded max loop value. This test make sure
    // infinitloop is improbable
    GTEST_ASSERT_TRUE(TestCanManager::g_callbackCounter == 5);
}

TEST(SUITE_NAME_CanManager, MsgSending)
{
    RoverCan2::CanDevice device(RoverCan2::Constant::eDeviceId::TEST_DEVICE);
    TestCanManager::CanDriverMock canDriver;
    RoverCan2::CanManager canManager(canDriver, device);
    canManager.init();
    canManager.update();

    RoverCan2::Msgs::TestMsg msg;
    msg.data().closeLoop = false;
    msg.data().cmd = 69.0F;

    canManager.sendMsg(RoverCan2::Constant::eDeviceId::TEST_DEVICE, msg);

    GTEST_ASSERT_TRUE(canDriver.msgSentBuffer.size() == 2U);

    auto msgOpt = canDriver.msgSentBuffer.getValue();
    GTEST_ASSERT_TRUE(msgOpt.has_value());
    GTEST_ASSERT_TRUE(msgOpt.value().getCanID() == RoverCan2::Constant::eDeviceId::TEST_DEVICE);
    GTEST_ASSERT_TRUE(msgOpt.value().getMsgID() == RoverCan2::Constant::eMsgId::TEST_MSG);
    GTEST_ASSERT_TRUE(msgOpt.value().getMsgContentID() == TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CMD));

    msgOpt = canDriver.msgSentBuffer.getValue();
    GTEST_ASSERT_TRUE(msgOpt.has_value());
    GTEST_ASSERT_TRUE(msgOpt.value().getCanID() == RoverCan2::Constant::eDeviceId::TEST_DEVICE);
    GTEST_ASSERT_TRUE(msgOpt.value().getMsgID() == RoverCan2::Constant::eMsgId::TEST_MSG);
    GTEST_ASSERT_TRUE(msgOpt.value().getMsgContentID() == TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CLOSE_LOOP));

    msgOpt = canDriver.msgSentBuffer.getValue();
    GTEST_ASSERT_TRUE(!msgOpt.has_value());
}

TEST(SUITE_NAME_CanManager, MsgSendingInvalidSenderID)
{
    TestCanManager::CanDriverMock canDriver;
    RoverCan2::CanManager canManager(canDriver);
    canManager.init();
    canManager.update();

    RoverCan2::Msgs::TestMsg msg;
    msg.data().closeLoop = false;
    msg.data().cmd = 69.0F;

    canManager.sendMsg(RoverCan2::Constant::eDeviceId::TEST_DEVICE, msg);

    GTEST_ASSERT_TRUE(canDriver.msgSentBuffer.size() == 0U);
}

TEST(SUITE_NAME_CanManager, MsgSendingInvalidSenderIDBypass)
{
    TestCanManager::CanDriverMock canDriver;
    RoverCan2::CanManager canManager(canDriver);
    canManager.init();
    canManager.update();

    RoverCan2::Msgs::TestMsg msg;
    msg.data().closeLoop = false;
    msg.data().cmd = 69.0F;

    canManager.sendMsg(RoverCan2::Constant::eDeviceId::TEST_DEVICE, msg, true);

    GTEST_ASSERT_TRUE(canDriver.msgSentBuffer.size() == 2U);

    auto msgOpt = canDriver.msgSentBuffer.getValue();
    GTEST_ASSERT_TRUE(msgOpt.has_value());
    GTEST_ASSERT_TRUE(msgOpt.value().getCanID() == RoverCan2::Constant::eDeviceId::TEST_DEVICE);
    GTEST_ASSERT_TRUE(msgOpt.value().getMsgID() == RoverCan2::Constant::eMsgId::TEST_MSG);
    GTEST_ASSERT_TRUE(msgOpt.value().getMsgContentID() == TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CMD));

    msgOpt = canDriver.msgSentBuffer.getValue();
    GTEST_ASSERT_TRUE(msgOpt.has_value());
    GTEST_ASSERT_TRUE(msgOpt.value().getCanID() == RoverCan2::Constant::eDeviceId::TEST_DEVICE);
    GTEST_ASSERT_TRUE(msgOpt.value().getMsgID() == RoverCan2::Constant::eMsgId::TEST_MSG);
    GTEST_ASSERT_TRUE(msgOpt.value().getMsgContentID() == TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CLOSE_LOOP));

    msgOpt = canDriver.msgSentBuffer.getValue();
    GTEST_ASSERT_TRUE(!msgOpt.has_value());
}
