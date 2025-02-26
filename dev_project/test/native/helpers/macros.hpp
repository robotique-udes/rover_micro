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
TEST(SUITE_HELPER_Macro, TO_UNDERLYING)
{
    ASSERT_EQ(TO_UNDERLYING(MacroEnumClassTest::ELEM_0), static_cast<size_t>(0));
}

// Test at compilation
TEST(SUITE_HELPER_Macro, _EVER)
{
    for (;;)
    {
        break;
    }

    ASSERT_TRUE(true);
}
