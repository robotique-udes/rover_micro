#include <gtest/gtest.h>
#include "rover_can2/helpers.hpp"

#include "rover_can2/msgs/test_msg.hpp"

// =============================================================================
// Suite
// =============================================================================
TEST(SUITE_ROVER_CAN2_Helpers, DATA_LENGTH_MATCHES_MSG_CONTENT_VALID)
{
    size_t expectedSize = sizeof(float) + TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA);

    ASSERT_TRUE(RoverCan2::Helpers::DATA_LENGTH_MATCHES_MSG_CONTENT<float>(expectedSize));
}

TEST(SUITE_ROVER_CAN2_Helpers, DATA_LENGTH_MATCHES_MSG_CONTENT_INVALID)
{
    size_t expectedSize = sizeof(float) + TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA);

    ASSERT_FALSE(RoverCan2::Helpers::DATA_LENGTH_MATCHES_MSG_CONTENT<float>(expectedSize + 1));
    ASSERT_FALSE(RoverCan2::Helpers::DATA_LENGTH_MATCHES_MSG_CONTENT<float>(expectedSize - 1));
    ASSERT_FALSE(RoverCan2::Helpers::DATA_LENGTH_MATCHES_MSG_CONTENT<float>(expectedSize - 20));
}

TEST(SUITE_ROVER_CAN2_Helpers, ROVER_MSG_CONTENT_TO_CAN_MSG)
{
    bool closedLoop = true;

    RoverCan2::CanMsg msg_;
    RoverCan2::Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(RoverCan2::Constant::eMsgId::TEST_MSG, 0x01, closedLoop, msg_);

    GTEST_ASSERT_TRUE(msg_.getMsgID() == RoverCan2::Constant::eMsgId::TEST_MSG);
    GTEST_ASSERT_TRUE(msg_.getMsgContentID() == 0x01);
    GTEST_ASSERT_TRUE(msg_.dataLength == sizeof(bool) + TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA));
    GTEST_ASSERT_TRUE(static_cast<bool>(msg_.msgData[TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA)]) == true);
}

TEST(SUITE_ROVER_CAN2_Helpers, ROVER_MSG_CONTENT_TO_CAN_MSG_FLOAT)
{
    const float cmd = 69.0f;
    RoverCan2::CanMsg msg_;
    RoverCan2::Helpers::ROVER_MSG_CONTENT_TO_CAN_MSG(RoverCan2::Constant::eMsgId::TEST_MSG,
                                                     TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CMD),
                                                     cmd,
                                                     msg_);

    // Verify message header fields
    GTEST_ASSERT_EQ(msg_.getMsgID(), RoverCan2::Constant::eMsgId::TEST_MSG);
    GTEST_ASSERT_EQ(msg_.getMsgContentID(), TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CMD));
    GTEST_ASSERT_EQ(msg_.dataLength, sizeof(float) + TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA));

    float expectedValue;
    std::memcpy(&expectedValue, &msg_.msgData[TO_UNDERLYING(RoverCan2::Constant::eDataIndex::START_OF_DATA)], sizeof(float));

    GTEST_ASSERT_TRUE(expectedValue == cmd);
}
TEST(SUITE_ROVER_CAN2_Helpers, CAN_MSG_TO_ROVER_MSG_CONTENT)
{
    RoverCan2::Msgs::TestMsg testMsg;
    testMsg.data().closeLoop = true;

    RoverCan2::CanMsg msg_ = testMsg.getCanMsg(testMsg.getMsgContentCount() - 1U).value();

    testMsg.data().closeLoop = false;
    RoverCan2::Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, testMsg.data().closeLoop);

    GTEST_ASSERT_TRUE(testMsg.data().closeLoop == true);
}

TEST(SUITE_ROVER_CAN2_Helpers, CAN_MSG_TO_ROVER_MSG_CONTENT_FLOAT)
{
    RoverCan2::Msgs::TestMsg testMsg;
    testMsg.data().cmd = 69.0F;

    RoverCan2::CanMsg msg_ = testMsg.getCanMsg(TO_UNDERLYING(RoverCan2::Msgs::TestMsg::eMsgContentID::CMD)).value();

    testMsg.data().closeLoop = false;
    RoverCan2::Helpers::CAN_MSG_TO_ROVER_MSG_CONTENT(msg_, testMsg.data().cmd);

    GTEST_ASSERT_TRUE(testMsg.data().cmd == 69.0F);
}
