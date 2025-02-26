#include <gtest/gtest.h>
#include "rover_lib2/helpers/deref_array.hpp"

TEST(SUITE_HELPER_DerefArray, DefaultConstruction)
{
    RoverLib2::DerefArray<int, 5> arr;
    ASSERT_FALSE(arr.valid());
}

TEST(SUITE_HELPER_DerefArray, ConstructWithBuffer)
{
    int buffer[5] = {1, 2, 3, 4, 5};
    RoverLib2::DerefArray<int, 5> arr(*buffer);
    ASSERT_TRUE(arr.valid());
    ASSERT_TRUE(arr.size() == 5);
    ASSERT_TRUE(arr[0] == 1);
    ASSERT_TRUE(arr[4] == 5);
}

TEST(SUITE_HELPER_DerefArray, AtFunction)
{
    int buffer[5] = {10, 20, 30, 40, 50};
    RoverLib2::DerefArray<int, 5> arr(*buffer);
    ASSERT_EQ(arr.at(2), 30);
}

TEST(SUITE_HELPER_DerefArray, FrontBack)
{
    int buffer[5] = {100, 200, 300, 400, 500};
    RoverLib2::DerefArray<int, 5> arr(*buffer);
    ASSERT_EQ(arr.front(), 100);
    ASSERT_EQ(arr.back(), 500);
}

TEST(SUITE_HELPER_DerefArray, Fill)
{
    int buffer[5] = {0};
    RoverLib2::DerefArray<int, 5> arr(*buffer);
    arr.fill(42);
    for (std::size_t i = 0; i < arr.size(); ++i)
    {
        ASSERT_EQ(arr[i], 42);
    }
}

TEST(SUITE_HELPER_DerefArray, Swap)
{
    int buffer1[5] = {1, 2, 3, 4, 5};
    int buffer2[5] = {6, 7, 8, 9, 10};
    RoverLib2::DerefArray<int, 5> arr1(*buffer1);
    RoverLib2::DerefArray<int, 5> arr2(*buffer2);
    arr1.swap(arr2);
    ASSERT_EQ(arr1[0], 6);
    ASSERT_EQ(arr2[0], 1);
}

TEST(SUITE_HELPER_DerefArray, Iterators)
{
    int buffer[3] = {1, 2, 3};
    RoverLib2::DerefArray<int, 3> arr(*buffer);
    int sum = 0;
    for (auto it = arr.begin(); it != arr.end(); ++it)
    {
        sum += *it;
    }
    ASSERT_EQ(sum, 6);
}

TEST(SUITE_HELPER_DerefArray, ReverseIterators)
{
    int buffer[3] = {1, 2, 3};
    RoverLib2::DerefArray<int, 3> arr(*buffer);
    int expected = 3;
    for (auto it = arr.rbegin(); it != arr.rend(); ++it)
    {
        ASSERT_EQ(*it, expected);
        expected--;
    }
}

TEST(SUITE_HELPER_DerefArray, Comparison)
{
    int buffer1[3] = {1, 2, 3};
    int buffer2[3] = {1, 2, 3};
    int buffer3[3] = {4, 5, 6};
    RoverLib2::DerefArray<int, 3> arr1(*buffer1);
    RoverLib2::DerefArray<int, 3> arr2(*buffer2);
    RoverLib2::DerefArray<int, 3> arr3(*buffer3);
    ASSERT_TRUE(arr1 == arr2);
    ASSERT_FALSE(arr1 == arr3);
    ASSERT_TRUE(arr1 < arr3);
}

TEST(SUITE_HELPER_DerefArray, NullDataAccess)
{
    RoverLib2::DerefArray<int, 5> arr;
    ASSERT_DEATH(arr.at(0), "Data pointer is null");
}

TEST(SUITE_HELPER_DerefArray, OutOfBoundsAccess)
{
    int buffer[5] = {0, 1, 2, 3, 4};
    RoverLib2::DerefArray<int, 5> arr(*buffer);
    ASSERT_DEATH(arr.at(5), "DerefArray::at: pos out of range");
}

TEST(SUITE_HELPER_DerefArray, IteratorUsageOnNull)
{
    RoverLib2::DerefArray<int, 5> arr;
    ASSERT_DEATH(arr.begin(), "Data pointer is null");
}

TEST(SUITE_HELPER_DerefArray, ReverseIteratorUsageOnNull)
{
    RoverLib2::DerefArray<int, 5> arr;
    ASSERT_DEATH(arr.rbegin(), "Data pointer is null");
}

TEST(SUITE_HELPER_DerefArray, SwapNull)
{
    RoverLib2::DerefArray<int, 5> arr1;
    RoverLib2::DerefArray<int, 5> arr2;
    arr1.swap(arr2);
    ASSERT_FALSE(arr1.valid());
    ASSERT_FALSE(arr2.valid());
}

TEST(SUITE_HELPER_DerefArray, SelfAssignment)
{
    int buffer[5] = {0, 1, 2, 3, 4};
    RoverLib2::DerefArray<int, 5> arr(*buffer);
    arr = arr;
    ASSERT_TRUE(arr.valid());
    ASSERT_EQ(arr.at(0), 0);
}

TEST(SUITE_HELPER_DerefArray, ModifyThroughIterator)
{
    int buffer[5] = {0, 1, 2, 3, 4};
    RoverLib2::DerefArray<int, 5> arr(*buffer);
    *arr.begin() = 99;
    ASSERT_EQ(arr.at(0), 99);
}

TEST(SUITE_HELPER_DerefArray, CompareWithNull)
{
    int buffer[5] = {0, 1, 2, 3, 4};
    RoverLib2::DerefArray<int, 5> arr1(*buffer);
    RoverLib2::DerefArray<int, 5> arr2;
    ASSERT_DEATH(arr1 == arr2, "Data pointer is null");
}
