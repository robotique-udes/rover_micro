#include <gtest/gtest.h>

#include "rover_lib2/helpers/moving_average.hpp"

// =============================================================================
// Test Suite
// =============================================================================
TEST(SUITE_HELPER_Average, Construction)
{
    MovingAverage<int8_t, 10> average(100);

    ASSERT_EQ(average.getAverage(), static_cast<float>(100));
}

TEST(SUITE_HELPER_Average, Adding_Value)
{
    MovingAverage<int8_t, 5> average(0);

    average.addValue(10);

    ASSERT_EQ(average.getAverage(), static_cast<float>(2));
}

TEST(SUITE_HELPER_Average, Full_Average_Table)
{
    MovingAverage<int8_t, 3> average(0);

    average.addValue(1);
    average.addValue(5);
    average.addValue(3);

    average.addValue(10);
    
    ASSERT_EQ(average.getAverage(), static_cast<float>(6));
}

TEST(SUITE_HELPER_Average, Adding_Negative_Value)
{
    MovingAverage<int8_t, 3> average(0);

    average.addValue(-5);
    average.addValue(-5);
    average.addValue(4);
    
    ASSERT_EQ(average.getAverage(), static_cast<float>(-2));
}

TEST(SUITE_HELPER_Average, Singular_Coefficient)
{
    MovingAverage<int8_t, 1> average(0);

    average.addValue(10);
    average.addValue(30);

    ASSERT_EQ(average.getAverage(), static_cast<float>(30));
}