#include <gtest/gtest.h>

#include "rover_lib2/helpers/one_shot_timer.hpp"

// =============================================================================
// Helpers
// =============================================================================
namespace TestOneShotTimer
{
    static uint64_t counter = 0UL;
    uint64_t mock_millis()
    {
        return counter;
    }
}  // namespace TestOneShotTimer

// =============================================================================
// Suite
// =============================================================================

TEST(SUITE_NAME_OneShotTimer, Construction)
{
    OneShotTimer<uint64_t, TestOneShotTimer::mock_millis> oneShotTimer(2);
}

TEST(SUITE_NAME_OneShotTimer, Triggers)
{
    OneShotTimer<uint64_t, TestOneShotTimer::mock_millis> oneShotTimer(2);

    GTEST_ASSERT_TRUE(oneShotTimer.isReady() == false);
    GTEST_ASSERT_TRUE(oneShotTimer.isReady() == false);
    TestOneShotTimer::counter = 3;
    GTEST_ASSERT_TRUE(oneShotTimer.isReady() == true);
    GTEST_ASSERT_TRUE(oneShotTimer.isReady() == false);

    TestOneShotTimer::counter = 10'000;
    GTEST_ASSERT_TRUE(oneShotTimer.isReady() == false);
}
