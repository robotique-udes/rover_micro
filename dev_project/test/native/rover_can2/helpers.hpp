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
