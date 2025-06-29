#include <gtest/gtest.h>

#include "rover_lib2/controllers/PID.hpp"

#include <thread>
#include <chrono>
#include <cmath>

// =============================================================================
// Helpers
// =============================================================================
namespace TestPID
{
    class MockTime
    {
      public:
        static uint64_t mockTime;
        static uint64_t micros()
        {
            return mockTime;
        }
        static void setTime(uint64_t time)
        {
            mockTime = time;
        }
        static void advanceTime(uint64_t microseconds)
        {
            mockTime += microseconds;
        }
    };

    uint64_t MockTime::mockTime = 0;

    // Helper to check if two floats are approximately equal
    bool isApproxEqual(float a, float b, float tolerance = 1e-6f)
    {
        return std::abs(a - b) < tolerance;
    }

    Controllers::PID createBasicPID()
    {
        return Controllers::PID(1.0f, 0.1f, 0.01f, 10.0f);
    }

}  // namespace TestPID

// =============================================================================
// Suite
// =============================================================================

TEST(SUITE_ROVER_LIB2_PID, Construction)
{
    Controllers::PID pid(1.0f, 0.5f, 0.1f, 5.0f, 20000ULL, 0.01f);
    Controllers::PID pidDefault(2.0f, 1.0f, 0.2f, 10.0f);
}

TEST(SUITE_ROVER_LIB2_PID, InitialState)
{
    Controllers::PID pid = TestPID::createBasicPID();

    // Initial command should be 0 when input equals target
    float result = pid.computeCommand(5.0f, 5.0f);
    EXPECT_FLOAT_EQ(result, 0.0f);
}

TEST(SUITE_ROVER_LIB2_PID, ProportionalControl)
{
    Controllers::PID pid(2.0f, 0.0f, 0.0f, 10.0f, 0ULL);

    // Test positive error (target > input)
    float result = pid.computeCommand(3.0f, 5.0f);
    EXPECT_FLOAT_EQ(result, 4.0f);  // kp * error = 2.0 * (5-3) = 4.0

    // Test negative error (target < input)
    pid.reset();
    result = pid.computeCommand(7.0f, 5.0f);
    EXPECT_FLOAT_EQ(result, -4.0f);  // kp * error = 2.0 * (5-7) = -4.0
}

TEST(SUITE_ROVER_LIB2_PID, IntegralControl)
{
    Controllers::PID pid(0.0f, 1.0f, 0.0f, 100.0f, 0ULL);  // Only integral term

    // First call - should accumulate integral
    float result1 = pid.computeCommand(0.0f, 10.0f);
    EXPECT_GT(result1, 0.0f);

    // Second call - integral should continue accumulating
    float result2 = pid.computeCommand(0.0f, 10.0f);
    EXPECT_GT(result2, result1);
}

TEST(SUITE_ROVER_LIB2_PID, IntegralWindupProtection)
{
    Controllers::PID pid(0.0f, 10.0f, 0.0f, 5.0f, 0ULL);  // High Ki, low integral limit

    // Apply large error multiple times to trigger windup protection
    for (int i = 0; i < 10; ++i)
    {
        pid.computeCommand(0.0f, 100.0f);
    }

    float result = pid.computeCommand(0.0f, 100.0f);
    EXPECT_LE(std::abs(result), 5.0f);  // Should be clamped to integral limit
}

TEST(SUITE_ROVER_LIB2_PID, DerivativeControl)
{
    Controllers::PID pid(0.0f, 0.0f, 1.0f, 10.0f, 0ULL);  // Only derivative term

    // First measurement - no derivative yet
    float result1 = pid.computeCommand(0.0f, 10.0f);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));  // Makes sure dt isn't 0

    // Second measurement with different error - should have derivative component
    float result2 = pid.computeCommand(2.0f, 10.0f);

    // Derivative term should respond to change in error
    EXPECT_NE(result1, result2);
}

TEST(SUITE_ROVER_LIB2_PID, CalcPeriodRespected)
{
    Controllers::PID pid(1.0f, 0.1f, 0.01f, 10.0f, 50000ULL);  // 50ms period

    float result1 = pid.computeCommand(0.0f, 10.0f);

    // Immediate second call should return same result (within period)
    float result2 = pid.computeCommand(5.0f, 10.0f);
    EXPECT_FLOAT_EQ(result1, result2);

    // After sufficient time delay, result should be different
    std::this_thread::sleep_for(std::chrono::microseconds(60000));
    float result3 = pid.computeCommand(5.0f, 10.0f);
    EXPECT_NE(result1, result3);
}

TEST(SUITE_ROVER_LIB2_PID, ErrorTolerance)
{
    Controllers::PID pid(1.0f, 0.1f, 0.01f, 10.0f, 0ULL, 0.1f);  // 0.1 error tolerance

    // Error within tolerance should return 0
    float result = pid.computeCommand(9.95f, 10.0f);
    EXPECT_FLOAT_EQ(result, 0.0f);

    // Error outside tolerance should return non-zero
    result = pid.computeCommand(9.8f, 10.0f);
    EXPECT_NE(result, 0.0f);
}

TEST(SUITE_ROVER_LIB2_PID, NaNHandling)
{
    Controllers::PID pid(1.0f, 0.1f, 0.01f, 10.0f, 0ULL);

    // Test NaN input
    float result = pid.computeCommand(std::numeric_limits<float>::quiet_NaN(), 10.0f);
    EXPECT_FALSE(std::isnan(result));

    // Test NaN target
    result = pid.computeCommand(5.0f, std::numeric_limits<float>::quiet_NaN());
    EXPECT_FALSE(std::isnan(result));
}

TEST(SUITE_ROVER_LIB2_PID, InfinityHandling)
{
    Controllers::PID pid(1.0f, 0.1f, 0.01f, 10.0f, 0ULL);

    // Test infinity input
    float result = pid.computeCommand(std::numeric_limits<float>::infinity(), 10.0f);
    EXPECT_FALSE(std::isnan(result));
    EXPECT_FALSE(std::isinf(result));

    // Test negative infinity
    result = pid.computeCommand(-std::numeric_limits<float>::infinity(), 10.0f);
    EXPECT_FALSE(std::isnan(result));
    EXPECT_FALSE(std::isinf(result));
}

TEST(SUITE_ROVER_LIB2_PID, Reset)
{
    Controllers::PID pid(1.0f, 1.0f, 1.0f, 10.0f, 0ULL);

    // Build up some state
    pid.computeCommand(0.0f, 10.0f);
    pid.computeCommand(2.0f, 10.0f);
    pid.computeCommand(4.0f, 10.0f);

    // Reset should clear internal state
    Controllers::PID freshPid(1.0f, 1.0f, 1.0f, 10.0f, 0ULL);
    pid.reset();

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    // After reset, response should be same as initial response
    float resetResult = pid.computeCommand(5.0f, 10.0f);
    float freshResult = freshPid.computeCommand(5.0f, 10.0f);

    GTEST_ASSERT_TRUE(IN_ERROR(resetResult, 0.1F, freshResult));
}

TEST(SUITE_ROVER_LIB2_PID, SetGains)
{
    Controllers::PID pid(1.0f, 0.0f, 0.0f, 10.0f, 0ULL);

    // Test with initial gains
    float result1 = pid.computeCommand(0.0f, 10.0f);

    pid.reset();

    // Change gains and test
    pid.setGains(2.0f, 0.0f, 0.0f);
    float result2 = pid.computeCommand(0.0f, 10.0f);

    // Result should be doubled due to doubled Kp
    EXPECT_FLOAT_EQ(result2, result1 * 2.0f);
}

TEST(SUITE_ROVER_LIB2_PID, ZeroTimeStep)
{
    Controllers::PID pid(1.0f, 0.1f, 1.0f, 10.0f, 0ULL);

    // When dt is effectively zero, derivative term should be zero
    float result = pid.computeCommand(0.0f, 10.0f);
    EXPECT_FALSE(std::isnan(result));
    EXPECT_FALSE(std::isinf(result));
}

TEST(SUITE_ROVER_LIB2_PID, NegativeIntegralLimit)
{
    // Constructor should take absolute value of integral limit
    Controllers::PID pid(0.0f, 1.0f, 0.0f, -5.0f, 0ULL);

    // Apply large positive error
    for (int i = 0; i < 20; ++i)
    {
        pid.computeCommand(0.0f, 100.0f);
    }

    float positiveResult = pid.computeCommand(0.0f, 100.0f);
    EXPECT_LE(positiveResult, 5.0f);

    pid.reset();

    // Apply large negative error
    for (int i = 0; i < 20; ++i)
    {
        pid.computeCommand(100.0f, 0.0f);
    }

    float negativeResult = pid.computeCommand(100.0f, 0.0f);
    EXPECT_GE(negativeResult, -5.0f);
}

TEST(SUITE_ROVER_LIB2_PID, EdgeCaseGains)
{
    // Test with zero gains
    Controllers::PID pidZero(0.0f, 0.0f, 0.0f, 10.0f, 0ULL);
    float result = pidZero.computeCommand(0.0f, 10.0f);
    EXPECT_FLOAT_EQ(result, 0.0f);

    // Test with very small gains
    Controllers::PID pidSmall(1e-6f, 1e-6f, 1e-6f, 10.0f, 0ULL);
    result = pidSmall.computeCommand(0.0f, 10.0f);
    EXPECT_FALSE(std::isnan(result));

    // Test with large gains
    Controllers::PID pidLarge(1000.0f, 100.0f, 10.0f, 1000.0f, 0ULL);
    result = pidLarge.computeCommand(0.0f, 1.0f);
    EXPECT_FALSE(std::isnan(result));
    EXPECT_FALSE(std::isinf(result));
}
