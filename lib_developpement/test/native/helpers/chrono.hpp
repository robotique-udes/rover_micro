#include <gtest/gtest.h>

#include "rover_lib2/helpers/chrono.hpp"
#include <time.h>

// =============================================================================
// Helpers
// =============================================================================
static uint64_t counter = 0UL;
uint64_t mock_millis()
{
    return counter;
}

void increase_millis(void)
{
    counter++;
}

// =============================================================================
// Test Suite
// =============================================================================
TEST(SUITE_HELPER_Chrono, Construction)
{
    increase_millis();
    increase_millis();
    Chrono<uint64_t, mock_millis> chrono;

    ASSERT_EQ(chrono.getTime(), static_cast<uint64_t>(0));
}

TEST(SUITE_HELPER_Chrono, Accumulate_When_Not_Started)
{
    Chrono<uint64_t, mock_millis> chrono;

    increase_millis();
    increase_millis();
    ASSERT_EQ(chrono.getTime(), static_cast<uint64_t>(2));
}

TEST(SUITE_HELPER_Chrono, Start_Resets_Chrono)
{
    Chrono<uint64_t, mock_millis> chrono;
    increase_millis();
    increase_millis();

    chrono.start();

    increase_millis();
    increase_millis();
    ASSERT_EQ(chrono.getTime(), static_cast<uint64_t>(2));
}

TEST(SUITE_HELPER_Chrono, Pause_Works)
{
    Chrono<uint64_t, mock_millis> chrono;

    increase_millis();
    increase_millis();
    chrono.pause();
    increase_millis();

    ASSERT_EQ(chrono.getTime(), static_cast<uint64_t>(2));
}

TEST(SUITE_HELPER_Chrono, Pause_Resume_Works)
{
    Chrono<uint64_t, mock_millis> chrono;
    increase_millis();
    increase_millis();

    chrono.pause();
    increase_millis();
    increase_millis();

    chrono.resume();
    increase_millis();
    increase_millis();

    ASSERT_EQ(chrono.getTime(), static_cast<uint64_t>(4));
}

TEST(SUITE_HELPER_Chrono, Restart_Works)
{
    Chrono<uint64_t, mock_millis> chrono;
    increase_millis();
    increase_millis();

    chrono.restart();
    increase_millis();
    increase_millis();

    ASSERT_EQ(chrono.getTime(), static_cast<uint64_t>(2));
}
