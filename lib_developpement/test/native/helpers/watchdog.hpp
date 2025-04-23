#include <gtest/gtest.h>

#include "rover_lib2/helpers/watchdog.hpp"

// =============================================================================
// Helpers
// =============================================================================
namespace TestWatchdog
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

}  // namespace TestWatchdog

// =============================================================================
// Suite
// =============================================================================

TEST(SUITE_NAME_Watchdog, Construction)
{
    Watchdog<uint64_t, TestWatchdog::mock_millis> watchdog(500);
}

TEST(SUITE_NAME_Watchdog, Trigger)
{
    Watchdog<uint64_t, TestWatchdog::mock_millis> watchdog(50);

    for (size_t i = 0; i < 51; i++)
    {
        TestWatchdog::increase_millis();
    }

    GTEST_ASSERT_TRUE(watchdog.isOk() == false);
}

TEST(SUITE_NAME_Watchdog, Reset)
{
    Watchdog<uint64_t, TestWatchdog::mock_millis> watchdog(50);

    for (size_t i = 0; i < 25; i++)
    {
        TestWatchdog::increase_millis();
    }

    watchdog.reset();

    for (size_t i = 0; i < 26; i++)
    {
        TestWatchdog::increase_millis();
    }

    GTEST_ASSERT_TRUE(watchdog.isOk() == true);
}
