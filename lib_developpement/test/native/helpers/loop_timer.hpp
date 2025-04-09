#include <gtest/gtest.h>

#include "rover_lib2/helpers/loop_timer.hpp"

// =============================================================================
// Helpers
// =============================================================================
namespace TestLoopTimer
{
    static uint64_t counter = 0UL;
    uint64_t mock_millis()
    {
        return counter;
    }

    void increase_millis(void)
    {
        counter++;
    }

}  // namespace TestLoopTimer

// =============================================================================
// Suite
// =============================================================================

TEST(SUITE_NAME_LoopTimer, Construction)
{
    LoopTimer<uint64_t, TestLoopTimer::mock_millis> timer(10);
}

TEST(SUITE_NAME_LoopTimer, isReady_Normal)
{
    LoopTimer<uint64_t, TestLoopTimer::mock_millis> timer(10);
    TestLoopTimer::counter = 0ULL;

    TestLoopTimer::counter += 10ULL;
    GTEST_ASSERT_TRUE(timer.isReady());
    GTEST_ASSERT_TRUE(!timer.isReady());

    TestLoopTimer::counter += 10ULL;
    GTEST_ASSERT_TRUE(timer.isReady());
    GTEST_ASSERT_TRUE(!timer.isReady());
}

TEST(SUITE_NAME_LoopTimer, isReady_JitterCompensation)
{
    LoopTimer<uint64_t, TestLoopTimer::mock_millis> timer(10);
    TestLoopTimer::counter = 0ULL;

    TestLoopTimer::counter += 11ULL;
    GTEST_ASSERT_TRUE(timer.isReady());
    GTEST_ASSERT_TRUE(!timer.isReady());

    TestLoopTimer::counter += 9ULL;
    GTEST_ASSERT_TRUE(timer.isReady());
    GTEST_ASSERT_TRUE(!timer.isReady());

    TestLoopTimer::counter += 9ULL;
    GTEST_ASSERT_TRUE(!timer.isReady());

    TestLoopTimer::counter += 10ULL;
    GTEST_ASSERT_TRUE(timer.isReady());
    GTEST_ASSERT_TRUE(!timer.isReady());
}

TEST(SUITE_NAME_LoopTimer, isReady_JitterOvershoot)
{
    LoopTimer<uint64_t, TestLoopTimer::mock_millis> timer(10);
    TestLoopTimer::counter = 0ULL;

    TestLoopTimer::counter = 25ULL;
    GTEST_ASSERT_TRUE(timer.isReady());
    GTEST_ASSERT_TRUE(!timer.isReady());
}

TEST(SUITE_NAME_LoopTimer, isReady_ClockOvershoot)
{
    LoopTimer<uint64_t, TestLoopTimer::mock_millis> timer(10);
    TestLoopTimer::counter = 0ULL;

    TestLoopTimer::counter = 25ULL;
    GTEST_ASSERT_TRUE(timer.isReady());
    GTEST_ASSERT_TRUE(!timer.isReady());

    TestLoopTimer::counter = 10ULL;
    GTEST_ASSERT_TRUE(timer.isReady());
    GTEST_ASSERT_TRUE(!timer.isReady());

    TestLoopTimer::counter += 10'000ULL;
    GTEST_ASSERT_TRUE(timer.isReady());
    GTEST_ASSERT_TRUE(!timer.isReady());

    TestLoopTimer::counter += 3'000ULL;
    GTEST_ASSERT_TRUE(timer.isReady());
    GTEST_ASSERT_TRUE(!timer.isReady());
}
