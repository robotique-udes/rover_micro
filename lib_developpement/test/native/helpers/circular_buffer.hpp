#include <gtest/gtest.h>

#include "rover_lib2/helpers/circular_buffer.hpp"

// =============================================================================
// Test Suite
// =============================================================================
TEST(SUITE_HELPER_CircularBuffer, Construction)
{
    CircularBuffer<int, 10UL> buffer;
    ASSERT_TRUE(buffer.size() == 0);
    ASSERT_TRUE(buffer.getValue() == std::nullopt);
}

TEST(SUITE_HELPER_CircularBuffer, Add_Value)
{
    CircularBuffer<int, 10UL> buffer;
    int value_0 = 69;

    ASSERT_EQ(buffer.addValue(value_0), (CircularBuffer<int, 10UL>::eErrorCode::SUCCESS));
    ASSERT_TRUE(buffer.size() == 1);
    ASSERT_TRUE(buffer.getValue().value() == value_0);
}

TEST(SUITE_HELPER_CircularBuffer, Add_Value_Overflow)
{
    CircularBuffer<int, 5UL> buffer;
    for (int i = 0; i < 5; i++)
    {
        ASSERT_EQ(buffer.addValue(i), (CircularBuffer<int, 5UL>::eErrorCode::SUCCESS));
    }
    ASSERT_EQ(buffer.addValue(5), (CircularBuffer<int, 5UL>::eErrorCode::SUCCESS_DATA_LOSS));
    ASSERT_TRUE(buffer.size() == 5);
    ASSERT_TRUE(buffer.getValue().value() == 1);
}

TEST(SUITE_HELPER_CircularBuffer, FIFO)
{
    CircularBuffer<int, 5UL> buffer;
    for (int i = 0; i < 5; i++)
    {
        buffer.addValue(i);
    }
    for (int i = 0; i < 5; i++)
    {
        ASSERT_TRUE(buffer.getValue().value() == i);
    }
}

TEST(SUITE_HELPER_CircularBuffer, Empty)
{
    CircularBuffer<int, 5UL> buffer;
    buffer.emptyBuffer();
    ASSERT_TRUE(buffer.size() == 0);
    for (int i = 0; i < 5; i++)
    {
        buffer.addValue(i);
    }
    buffer.emptyBuffer();
    ASSERT_TRUE(buffer.size() == 0);
    for (int i = 0; i < 6; i++)
    {
        ASSERT_TRUE(!buffer.getValue());
    }
}

TEST(SUITE_HELPER_CircularBuffer, Add_Value_Error_Code)
{
    CircularBuffer<int, 3UL> buffer;
    ASSERT_EQ(buffer.addValue(1), (CircularBuffer<int, 3UL>::eErrorCode::SUCCESS));
    ASSERT_EQ(buffer.addValue(2), (CircularBuffer<int, 3UL>::eErrorCode::SUCCESS));
    ASSERT_EQ(buffer.addValue(3), (CircularBuffer<int, 3UL>::eErrorCode::SUCCESS));
    ASSERT_EQ(buffer.addValue(4), (CircularBuffer<int, 3UL>::eErrorCode::SUCCESS_DATA_LOSS));
}

TEST(SUITE_HELPER_CircularBuffer, Order_After_Overwrite)
{
    CircularBuffer<int, 3UL> buffer;
    buffer.addValue(1);
    buffer.addValue(2);
    buffer.addValue(3);
    buffer.addValue(4);
    ASSERT_TRUE(buffer.getValue().value() == 2);
    ASSERT_TRUE(buffer.getValue().value() == 3);
    ASSERT_TRUE(buffer.getValue().value() == 4);
}

TEST(SUITE_HELPER_CircularBuffer, Get_Empty_Buffer)
{
    CircularBuffer<int, 5UL> buffer;
    ASSERT_TRUE(buffer.getValue() == std::nullopt);
}

TEST(SUITE_HELPER_CircularBuffer, Empty_Multiple_Times)
{
    CircularBuffer<int, 5UL> buffer;
    buffer.addValue(1);
    buffer.emptyBuffer();
    buffer.emptyBuffer();
    buffer.emptyBuffer();
    ASSERT_TRUE(buffer.size() == 0);
    ASSERT_TRUE(buffer.getValue() == std::nullopt);
}

TEST(SUITE_HELPER_CircularBuffer, Handle_Non_Int_Types)
{
    CircularBuffer<std::string, 3UL> buffer;
    buffer.addValue("hello");
    buffer.addValue("world");
    ASSERT_TRUE(buffer.getValue().value() == "hello");
    ASSERT_TRUE(buffer.getValue().value() == "world");
}
