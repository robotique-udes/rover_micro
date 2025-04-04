/**
 * @file integration_tests.hpp
 * @brief This test suite is there to test the whole lib with more of a "user"
 * type of implementation instead of a developper one like other unit tests. Its
 * also some good example for an potential lib user
 *
 */
#include <gtest/gtest.h>

#include "rover_can2/can_device.hpp"
#include "rover_can2/can_manager.hpp"
#include "rover_can2/subscriber.hpp"
#include "rover_can2/drivers/can_driver_mock.hpp"

#include "rover_can2/msgs/test_msg.hpp"

// =============================================================================
// Helpers
// =============================================================================
namespace TestIntegrationTests
{
    bool closedLoopValue = false;
    float targetSpeedValue = 0.0F;

    class TestSystem : public RoverCan2::CanDevice<RoverCan2::SubscriberMember<RoverCan2::Msgs::TestMsg, TestSystem>>,
                       public RoverObject<TestSystem>
    {
      public:
        TestSystem():
            CanDevice<RoverCan2::SubscriberMember<RoverCan2::Msgs::TestMsg, TestSystem>>(
                RoverCan2::Constant::eDeviceId::TEST_DEVICE,
                RoverCan2::SubscriberMember{*this, &TestSystem::CB_testMsgs})
        {
        }

        void _init(void) {}

        void _update(void) {}

      private:
        void CB_testMsgs(const RoverCan2::Msgs::TestMsg& msg_)
        {
            closedLoopValue = msg_.getData().closeLoop;
            targetSpeedValue = msg_.getData().cmd;
        }
    };
}  // namespace TestIntegrationTests

// =============================================================================
// Suite
// =============================================================================

TEST(SUITE_ROVER_CAN2_IntegrationTests, IntegrationTestSuite)
{
    RoverCan2::Drivers::CanDriverMock canDriver;

    TestIntegrationTests::TestSystem system;
    system.init();
    system.update();

    RoverCan2::CanManager canManager(canDriver, system);
    canManager.init();

    // Simulating msgs comming in:
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
