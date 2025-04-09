/**
 * @file integration_tests.hpp
 * @brief This test suite is there to test the whole lib with more of a "user"
 * type of implementation instead of a developper one like other unit tests. Its
 * also some good example for an potential lib user
 *
 */
#include <gtest/gtest.h>

#include "rover_can2/device.hpp"
#include "rover_can2/manager.hpp"
#include "rover_can2/subscriber.hpp"
#include "rover_can2/publisher.hpp"
#include "rover_can2/drivers/driver_mock.hpp"

#include "rover_can2/msgs/test_msg.hpp"

// =============================================================================
// Helpers
// =============================================================================
namespace TestIntegrationTests
{
    constexpr float CMD_FOR_TEST_PUB_IN_CALLBACK = 100.0F;

    bool closedLoopValue = false;
    float targetSpeedValue = 0.0F;

    class TestSystem : public RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::TestMsg, TestSystem>,
                                                RoverCan2::Publisher<RoverCan2::Msgs::TestMsg2, 1>>,
                       public RoverObject<TestSystem>
    {
      public:
        TestSystem():
            Device(RoverCan2::Constant::eDeviceId::TEST_DEVICE,
                   RoverCan2::SubscriberMember{*this, &TestSystem::CB_testMsgs},
                   RoverCan2::Publisher<RoverCan2::Msgs::TestMsg2, 1>())
        {
        }

        void _init(void) {}

        void _update(void) {}

      private:
        void CB_testMsgs(const RoverCan2::Msgs::TestMsg& msg_)
        {
            closedLoopValue = msg_.getData().closeLoop;
            targetSpeedValue = msg_.getData().cmd;

            if (msg_.getData().cmd == CMD_FOR_TEST_PUB_IN_CALLBACK)
            {
                RoverCan2::Msgs::TestMsg2 msg;
                this->sendMsg(msg);
            }
        }
    };
}  // namespace TestIntegrationTests

// =============================================================================
// Suite
// =============================================================================

TEST(SUITE_ROVER_CAN2_IntegrationTests, MsgReception)
{
    HealthState::getInstance().setInError(false); // Disabling error state reporting to get accurate sentMsg buffer

    // Inits
    RoverCan2::Drivers::DriverMock canDriver;

    TestIntegrationTests::TestSystem system;
    system.init();
    system.update();

    RoverCan2::Manager canManager(canDriver, system);
    canManager.init();

    // Simulating one msgs comming in:
    RoverCan2::Msgs::TestMsg testMsg;
    testMsg.data().closeLoop = true;
    testMsg.data().cmd = 69.0F;
    for (uint8_t i = 0U; i < testMsg.getMsgContentCount(); i++)
    {
        if (auto canTestMsg = testMsg.getCanMsg(i))
        {
            canTestMsg.value().setCanID(RoverCan2::Constant::eDeviceId::TEST_DEVICE);
            canDriver.newMsgsBuffer.addValue(canTestMsg.value());
        }
    }

    GTEST_ASSERT_TRUE(TestIntegrationTests::closedLoopValue == false);
    GTEST_ASSERT_TRUE(TestIntegrationTests::targetSpeedValue == 0.0F);
    canManager.update();
    GTEST_ASSERT_TRUE(TestIntegrationTests::closedLoopValue == true);
    GTEST_ASSERT_TRUE(TestIntegrationTests::targetSpeedValue == 69.0F);
}

TEST(SUITE_ROVER_CAN2_IntegrationTests, MsgSending)
{
    // Inits
    RoverCan2::Drivers::DriverMock canDriver;

    TestIntegrationTests::TestSystem system;
    system.init();
    system.update();

    RoverCan2::Manager canManager(canDriver, system);
    canManager.init();

    RoverCan2::Msgs::TestMsg2 msg;
    system.sendMsg(msg);

    GTEST_ASSERT_TRUE(canDriver.msgSentBuffer.size() == 0);
    canManager.update();
    GTEST_ASSERT_TRUE(canDriver.msgSentBuffer.size() == TO_UNDERLYING(RoverCan2::Msgs::TestMsg2::eMsgContentID::eLAST));
}

TEST(SUITE_ROVER_CAN2_IntegrationTests, MsgSendingQueue)
{
    // Inits
    RoverCan2::Drivers::DriverMock canDriver;

    TestIntegrationTests::TestSystem system;
    system.init();
    system.update();

    RoverCan2::Manager canManager(canDriver, system);
    canManager.init();

    // Simulating one msgs comming in:
    RoverCan2::Msgs::TestMsg2 msg;
    system.sendMsg(msg);
    system.sendMsg(msg);
    system.sendMsg(msg);
    system.sendMsg(msg);

    GTEST_ASSERT_TRUE(canDriver.msgSentBuffer.size() == 0);
    canManager.update();
    GTEST_ASSERT_TRUE(canDriver.msgSentBuffer.size() == TO_UNDERLYING(RoverCan2::Msgs::TestMsg2::eMsgContentID::eLAST));
}

TEST(SUITE_ROVER_CAN2_IntegrationTests, MsgRecvAndSendFromCB)
{
    // Inits
    RoverCan2::Drivers::DriverMock canDriver;

    TestIntegrationTests::TestSystem system;
    system.init();
    system.update();

    RoverCan2::Manager canManager(canDriver, system);
    canManager.init();

    // Simulating one msgs comming in:
    RoverCan2::Msgs::TestMsg testMsg;
    testMsg.data().closeLoop = true;
    testMsg.data().cmd = TestIntegrationTests::CMD_FOR_TEST_PUB_IN_CALLBACK;
    for (uint8_t i = 0U; i < testMsg.getMsgContentCount(); i++)
    {
        if (auto canTestMsg = testMsg.getCanMsg(i))
        {
            canTestMsg.value().setCanID(RoverCan2::Constant::eDeviceId::TEST_DEVICE);
            canDriver.newMsgsBuffer.addValue(canTestMsg.value());
        }
    }

    GTEST_ASSERT_TRUE(canDriver.msgSentBuffer.size() == 0);
    canManager.update();
    GTEST_ASSERT_TRUE(canDriver.msgSentBuffer.size() == TO_UNDERLYING(RoverCan2::Msgs::TestMsg2::eMsgContentID::eLAST));
}
// Simulating one msgs comming in that should trigger a publish in it's callback:
