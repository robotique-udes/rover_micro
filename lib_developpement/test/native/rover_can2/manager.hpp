#include <gtest/gtest.h>

#include "rover_can2/manager/manager_slave.hpp"
#include "rover_can2/drivers/driver_mock.hpp"
#include "rover_can2/msgs/test_msg.hpp"
#include "rover_can2/msgs/error_state.hpp"

// =============================================================================
// Helpers
// =============================================================================
namespace TestCanManager
{
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

TEST(SUITE_ROVER_CAN2_CanManager, Construction)
{
    RoverCan2::Drivers::DriverMock canDriver;
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanManager::CB_Helper)> sub0(
        TestCanManager::CB_Helper);
    RoverCan2::Device device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, sub0);

    RoverCan2::ManagerSlave canManager(canDriver, device);
}

/**
 * @brief Makes sure the init class calls the driver's init
 *
 */
TEST(SUITE_ROVER_CAN2_CanManager, Init)
{
    RoverCan2::Drivers::DriverMock canDriver;
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanManager::CB_Helper)> sub0(
        TestCanManager::CB_Helper);
    RoverCan2::Device device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, sub0);

    RoverCan2::ManagerSlave canManager(canDriver, device);
    canManager.init();

    GTEST_ASSERT_TRUE(canDriver.isInited == true);
}

/**
 * @brief Makes sure the init class calls the driver's update
 *
 */
TEST(SUITE_ROVER_CAN2_CanManager, Update)
{
    RoverCan2::Drivers::DriverMock canDriver;
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanManager::CB_Helper)> sub0(
        TestCanManager::CB_Helper);
    RoverCan2::Device device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, sub0);

    RoverCan2::ManagerSlave canManager(canDriver, device);
    canManager.init();
    canManager.update();

    GTEST_ASSERT_TRUE(canDriver.hasUpdated == true);
}

TEST(SUITE_ROVER_CAN2_CanManager, MsgRecvHandling)
{
    RoverCan2::Drivers::DriverMock canDriver;
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanManager::CB_Helper)> sub0(
        TestCanManager::CB_Helper);
    RoverCan2::Device device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, sub0);

    TestCanManager::g_callbackCounter = 0U;
    RoverCan2::ManagerSlave canManager(canDriver, device);
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

TEST(SUITE_ROVER_CAN2_CanManager, MsgRecvHandlingInfiniteLoop)
{
    RoverCan2::Drivers::DriverMock canDriver;
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanManager::CB_Helper)> sub0(
        TestCanManager::CB_Helper);
    RoverCan2::Device device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, sub0);

    TestCanManager::g_callbackCounter = 0U;
    RoverCan2::ManagerSlave canManager(canDriver, device);
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
    GTEST_ASSERT_TRUE(TestCanManager::g_callbackCounter == 10UL);
}

TEST(SUITE_ROVER_CAN2_CanManager, MsgSending)
{
    RoverCan2::Device device(RoverCan2::Constant::eDeviceId::TEST_DEVICE);
    RoverCan2::Drivers::DriverMock canDriver;
    RoverCan2::ManagerSlave canManager(canDriver, device);
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

TEST(SUITE_ROVER_CAN2_CanManager, MsgSendingInvalidSenderID)
{
    RoverCan2::Drivers::DriverMock canDriver;
    RoverCan2::ManagerSlave canManager(canDriver);
    canManager.init();
    canManager.update();

    RoverCan2::Msgs::TestMsg msg;
    msg.data().closeLoop = false;
    msg.data().cmd = 69.0F;

    canManager.sendMsg(RoverCan2::Constant::eDeviceId::TEST_DEVICE, msg);

    GTEST_ASSERT_TRUE(canDriver.msgSentBuffer.size() == 0U);
}

TEST(SUITE_ROVER_CAN2_CanManager, MsgSendingInvalidSenderIDBypass)
{
    RoverCan2::Drivers::DriverMock canDriver;
    RoverCan2::ManagerSlave canManager(canDriver);
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

TEST(SUITE_ROVER_CAN2_CanManager, ErrorStateReportingNoDevices)
{
    RoverCan2::Drivers::DriverMock canDriver;
    RoverCan2::ManagerSlave canManager(canDriver);
    canManager.init();
    canManager.update();

    RoverCan2::Msgs::ErrorState msg;
    RoverCan2::CanMsg canMsg = msg.getCanMsg(msg.getMsgContentCount() - 1U).value();
    canDriver.newMsgsBuffer.addValue(canMsg);

    GTEST_ASSERT_TRUE(canDriver.msgSentBuffer.size() == 0);
    canManager.update();
    GTEST_ASSERT_TRUE(canDriver.msgSentBuffer.size() == 0);
}

TEST(SUITE_ROVER_CAN2_CanManager, ErrorStateReceivedFromNonMaster)
{
    RoverCan2::Drivers::DriverMock canDriver;
    RoverCan2::Device<> device1(RoverCan2::Constant::eDeviceId::TEST_DEVICE);

    RoverCan2::ManagerSlave canManager(canDriver, device1);
    canManager.init();
    canManager.update();

    RoverCan2::Msgs::ErrorState msg;
    RoverCan2::CanMsg canMsg = msg.getCanMsg(msg.getMsgContentCount() - 1U).value();
    canDriver.newMsgsBuffer.addValue(canMsg);

    GTEST_ASSERT_TRUE(canDriver.msgSentBuffer.size() == 0);
    canManager.update();
    GTEST_ASSERT_TRUE(canDriver.msgSentBuffer.size() == 0);
}

TEST(SUITE_ROVER_CAN2_CanManager, ErrorStateReportingWithDevices)
{
    RoverCan2::Drivers::DriverMock canDriver;
    RoverCan2::Device<> device1(RoverCan2::Constant::eDeviceId::TEST_DEVICE);
    RoverCan2::Device<> device2(RoverCan2::Constant::eDeviceId::TEST_DEVICE);
    RoverCan2::Device<> device3(RoverCan2::Constant::eDeviceId::TEST_DEVICE);

    RoverCan2::ManagerSlave canManager(canDriver, device1, device2, device3);
    canManager.init();
    canManager.update();

    // Simulate ErrorState msg reception from Master, manager should make all
    // registered devices answer with their ErrorState
    RoverCan2::Msgs::ErrorState msg;
    RoverCan2::CanMsg canMsg = msg.getCanMsg(msg.getMsgContentCount() - 1U).value();
    canMsg.setCanID(RoverCan2::Constant::eDeviceId::MASTER_COMPUTER_UNIT);
    canDriver.newMsgsBuffer.addValue(canMsg);

    GTEST_ASSERT_TRUE(canDriver.msgSentBuffer.size() == 0);
    canManager.update();
    uint8_t nbOfMessageExpected = 3U * msg.getMsgContentCount();
    GTEST_ASSERT_TRUE(canDriver.msgSentBuffer.size() == nbOfMessageExpected);
}

TEST(SUITE_ROVER_CAN2_CanManager, ManagerDeviceManagementIntegrationTest)
{
    RoverCan2::Publisher<RoverCan2::Msgs::TestMsg, 10UL> pub0;
    RoverCan2::Publisher<RoverCan2::Msgs::TestMsg, 10UL> pub1;

    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanManager::CB_Helper)> sub0(
        TestCanManager::CB_Helper);
    RoverCan2::SubscriberStandalone<RoverCan2::Msgs::TestMsg, decltype(TestCanManager::CB_Helper)> sub1(
        TestCanManager::CB_Helper);

    RoverCan2::Device device(RoverCan2::Constant::eDeviceId::TEST_DEVICE, pub0, pub1, sub0, sub1);

    RoverCan2::Drivers::DriverMock driver;
    RoverCan2::ManagerSlave manager(driver, device);
    manager.init();

    // Send msg
    RoverCan2::Msgs::TestMsg sendMsg;
    sendMsg.data().cmd = 69.0F;
    sendMsg.data().closeLoop = true;
    pub0.queueMsg(sendMsg);
    pub0.queueMsg(sendMsg);
    pub1.queueMsg(sendMsg);

    // Simulated Msg Reception
    std::array<uint8_t, sizeof(bool) + TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA)> data
        = {/*MsgID=TEST_MSG*/ TO_UNDERLYING(RoverCan2::Constant::eMsgId::TEST_MSG),
           /*MSG_CONTENT_ID=CMD*/ TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CLOSE_LOOP),
           /* DATA 0 = true */ 0x01};
    RoverCan2::CanMsg recvMsg(RoverCan2::Constant::eDeviceId::TEST_DEVICE, data.data(), data.size());
    driver.newMsgsBuffer.addValue(recvMsg);
    driver.newMsgsBuffer.addValue(recvMsg);
    driver.newMsgsBuffer.addValue(recvMsg);

    TestCanManager::g_callbackCounter = 0UL;
    GTEST_ASSERT_TRUE(TestCanManager::g_callbackCounter == 0UL);
    GTEST_ASSERT_TRUE(driver.msgSentBuffer.size() == 0);
    manager.update();
    GTEST_ASSERT_TRUE(TestCanManager::g_callbackCounter == 3 * (TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::eLAST)));
    GTEST_ASSERT_TRUE(driver.msgSentBuffer.size() == 3 * (TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::eLAST)));
}

TEST(SUITE_ROVER_CAN2_CanManager, ManagerHealthStateReporting)
{
    RoverCan2::Device device(RoverCan2::Constant::eDeviceId::TEST_DEVICE);

    RoverCan2::Drivers::DriverMock driver;
    RoverCan2::ManagerSlave manager(driver, device);
    manager.init();
    HealthState::getInstance().setInError();

    // Error state should be reported each 2 seconds
    // std::this_thread::sleep_for(std::chrono::seconds(2));
    manager.update();

    auto msgOpt = driver.msgSentBuffer.getValue();
    GTEST_ASSERT_TRUE(msgOpt.has_value());  // Msg was appended

    GTEST_ASSERT_TRUE(msgOpt.value().getCanID() == RoverCan2::Constant::eDeviceId::TEST_DEVICE);
    GTEST_ASSERT_TRUE(msgOpt.value().getMsgID() == RoverCan2::Constant::eMsgId::ERROR_STATE);
}
