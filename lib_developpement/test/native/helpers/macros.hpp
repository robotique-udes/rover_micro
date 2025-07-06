#include <gtest/gtest.h>
#include "rover_lib2/helpers/macros.hpp"

// =============================================================================
// Helpers
// =============================================================================
enum class MacroEnumClassTest : size_t
{
    ELEM_0,
    ELEM_1,
    ELEM_2,
    ELEM_3,
    ELEM_4,
};

// =============================================================================
// Test Suite
// =============================================================================
TEST(SUITE_HELPER_Macro, _TO_UNDERLYING)
{
    ASSERT_EQ(TO_UNDERLYING(MacroEnumClassTest::ELEM_0), static_cast<size_t>(0));
}

// Test at compilation
TEST(SUITE_HELPER_Macro, _EVER)
{
    for (EVER)
    {
        break;
    }

    ASSERT_TRUE(true);
}

TEST(SUITE_HELPER_Macro, _ABS)
{
    float valueFloat = 1.0F;
    GTEST_ASSERT_TRUE(ABS(valueFloat) == 1.0F);
    valueFloat = -1.0F;
    GTEST_ASSERT_TRUE(ABS(valueFloat) == 1.0F);

    int32_t valueInt = 1;
    GTEST_ASSERT_TRUE(ABS(valueInt) == 1);
    valueInt = -1;
    GTEST_ASSERT_TRUE(ABS(valueInt) == 1);

    uint32_t valueUint = 1UL;
    GTEST_ASSERT_TRUE(ABS(valueUint) == 1UL);
    valueUint = static_cast<uint32_t>(-1);
    GTEST_ASSERT_TRUE(ABS(valueUint) == static_cast<uint32_t>(-1));
}

TEST(SUITE_HELPER_Macro, _IN_ERROR)
{
    float mesurement = 0.0F;
    float target = 1.0F;
    float error = 0.5F;

    mesurement = -1.0F;
    GTEST_ASSERT_FALSE(IN_ERROR(mesurement, error, target));

    mesurement = -0.2F;
    GTEST_ASSERT_FALSE(IN_ERROR(mesurement, error, target));

    mesurement = 0.0F;
    GTEST_ASSERT_FALSE(IN_ERROR(mesurement, error, target));

    mesurement = 0.25F;
    GTEST_ASSERT_FALSE(IN_ERROR(mesurement, error, target));

    mesurement = 0.5F;
    GTEST_ASSERT_TRUE(IN_ERROR(mesurement, error, target));

    mesurement = 0.75F;
    GTEST_ASSERT_TRUE(IN_ERROR(mesurement, error, target));

    mesurement = 1.0F;
    GTEST_ASSERT_TRUE(IN_ERROR(mesurement, error, target));

    mesurement = 1.25F;
    GTEST_ASSERT_TRUE(IN_ERROR(mesurement, error, target));

    mesurement = 1.50F;
    GTEST_ASSERT_TRUE(IN_ERROR(mesurement, error, target));

    mesurement = 1.75F;
    GTEST_ASSERT_FALSE(IN_ERROR(mesurement, error, target));
}

TEST(SUITE_HELPER_Macro, _CONSTRAIN)
{
    const float min = -2.0F;
    const float max = 2.0F;

    GTEST_ASSERT_TRUE(CONSTRAIN(0.0F, min, max) == 0.0F);
    GTEST_ASSERT_TRUE(CONSTRAIN(-3.0F, min, max) == -2.0F);
    GTEST_ASSERT_TRUE(CONSTRAIN(3.0F, min, max) == 2.0F);
}

TEST(SUITE_HELPER_Macro, _ROUND)
{
    GTEST_ASSERT_TRUE(ROUND(3.4) == 3.0);
    GTEST_ASSERT_TRUE(ROUND(3.5) == 4.0);  // Halfway case rounds up
    GTEST_ASSERT_TRUE(ROUND(-2.3) == -2.0);
    GTEST_ASSERT_TRUE(ROUND(-2.6) == -3.0);
    GTEST_ASSERT_TRUE(ROUND(0.0) == 0.0);
}

TEST(SUITE_HELPER_Macro, _ROUND_UP)
{
    EXPECT_DOUBLE_EQ(ROUND_UP(3.1), 4.0);
    EXPECT_DOUBLE_EQ(ROUND_UP(-2.9), -2.0);  // Towards +∞ (less negative)
    EXPECT_DOUBLE_EQ(ROUND_UP(0.0), 0.0);
    EXPECT_DOUBLE_EQ(ROUND_UP(5.0), 5.0);  // Exact integer remains unchanged
}

TEST(SUITE_HELPER_Macro, _ROUND_DOWN)
{
    EXPECT_DOUBLE_EQ(ROUND_DOWN(3.9), 3.0);
    EXPECT_DOUBLE_EQ(ROUND_DOWN(-2.1), -3.0);  // Towards -∞ (more negative)
    EXPECT_DOUBLE_EQ(ROUND_DOWN(0.0), 0.0);
    EXPECT_DOUBLE_EQ(ROUND_DOWN(5.0), 5.0);  // Exact integer remains unchanged
}

TEST(SUITE_HELPER_Macro, _MAP)
{
    EXPECT_FLOAT_EQ(MAP(5.0F, 0.0F, 10.0F, 0.0F, 100.0F), 50.0F);
    EXPECT_FLOAT_EQ(MAP(0.0F, 0.0F, 10.0F, 0.0F, 100.0F), 0.0F);
    EXPECT_FLOAT_EQ(MAP(10.0F, 0.0F, 10.0F, 0.0F, 100.0F), 100.0F);
    EXPECT_FLOAT_EQ(MAP(-5.0F, -10.0F, 0.0F, 0.0F, 100.0F), 50.0F);
    EXPECT_FLOAT_EQ(MAP(0.0F, 0.0F, 10.0F, 100.0F, 0.0F), 100.0F);
    EXPECT_FLOAT_EQ(MAP(10.0F, 0.0F, 10.0F, 100.0F, 0.0F), 0.0F);
    EXPECT_FLOAT_EQ(MAP(15.0F, 0.0F, 10.0F, 0.0F, 100.0F), 150.0F);
    EXPECT_FLOAT_EQ(MAP(-5.0F, 0.0F, 10.0F, 0.0F, 100.0F), -50.0F);
}
