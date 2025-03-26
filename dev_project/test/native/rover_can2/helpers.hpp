#include <gtest/gtest.h>
#include "rover_can2/helpers.hpp"

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
