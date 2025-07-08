/**
 * @author AI Autogen
 *
 */

#include <gtest/gtest.h>

#include "rover_lib2/filters/low_pass_EMA.hpp"
#include <cmath>
#include <vector>

// =============================================================================
// Helpers
// =============================================================================
namespace TestLowPassEMA
{
    constexpr float TOLERANCE = 1e-5f;

    // Helper to check if two floating point values are approximately equal
    bool isApproxEqual(const float& a, const float& b, const float& tolerance = TOLERANCE)
    {
        return std::abs(a - b) < tolerance;
    }

    // Generate step input sequence
    std::vector<float> generateStepInput(const std::size_t length, const float& stepValue = 1.0f)
    {
        return std::vector<float>(length, stepValue);
    }

    // Generate ramp input sequence
    std::vector<float> generateRampInput(const std::size_t length, const float& slope = 1.0f)
    {
        std::vector<float> ramp(length);
        for (std::size_t i = 0; i < length; ++i)
        {
            ramp[i] = static_cast<float>(i) * slope;
        }
        return ramp;
    }

}  // namespace TestLowPassEMA

// =============================================================================
// Suite
// =============================================================================

TEST(UITE_ROVER_LIB2_LowPassEMA, Construction)
{
    // Valid construction with default values
    Filters::LowPassEMA filter1;
    EXPECT_FLOAT_EQ(filter1.getFilteredValue(), 0.0f);

    // Valid construction with custom values
    Filters::LowPassEMA filter2(0.5f, 10.0f);
    EXPECT_FLOAT_EQ(filter2.getFilteredValue(), 10.0f);

    // Edge cases - valid boundary values
    Filters::LowPassEMA filterMin(0.0f, 5.0f);
    EXPECT_FLOAT_EQ(filterMin.getFilteredValue(), 5.0f);

    Filters::LowPassEMA filterMax(1.0f, 5.0f);
    EXPECT_FLOAT_EQ(filterMax.getFilteredValue(), 5.0f);
}

TEST(UITE_ROVER_LIB2_LowPassEMA, AlphaClamping)
{
    // Test alpha values outside valid range get clamped
    Filters::LowPassEMA filterNegative(-0.5f, 0.0f);
    Filters::LowPassEMA filterTooHigh(1.5f, 0.0f);

    // Should still construct successfully due to clamping
    EXPECT_FLOAT_EQ(filterNegative.getFilteredValue(), 0.0f);
    EXPECT_FLOAT_EQ(filterTooHigh.getFilteredValue(), 0.0f);

    // Test behavior - negative alpha should behave like alpha=0 (no response)
    const float result1 = filterNegative.addValue(10.0f);
    EXPECT_FLOAT_EQ(result1, 0.0f);  // Should remain at initial value

    // Test behavior - alpha>1 should behave like alpha=1 (direct pass-through)
    const float result2 = filterTooHigh.addValue(10.0f);
    EXPECT_FLOAT_EQ(result2, 10.0f);  // Should jump to new value
}

TEST(UITE_ROVER_LIB2_LowPassEMA, BasicFiltering)
{
    Filters::LowPassEMA filter(0.5f, 0.0f);

    // First value: y[0] = 0.5 * 10 + 0.5 * 0 = 5
    const float result1 = filter.addValue(10.0f);
    EXPECT_FLOAT_EQ(result1, 5.0f);
    EXPECT_FLOAT_EQ(filter.getFilteredValue(), 5.0f);

    // Second value: y[1] = 0.5 * 10 + 0.5 * 5 = 7.5
    const float result2 = filter.addValue(10.0f);
    EXPECT_FLOAT_EQ(result2, 7.5f);

    // Third value: y[2] = 0.5 * 10 + 0.5 * 7.5 = 8.75
    const float result3 = filter.addValue(10.0f);
    EXPECT_FLOAT_EQ(result3, 8.75f);
}

TEST(UITE_ROVER_LIB2_LowPassEMA, StepResponse)
{
    Filters::LowPassEMA filter(0.2f, 0.0f);
    const auto stepInput = TestLowPassEMA::generateStepInput(50, 1.0f);  // More samples for convergence

    std::vector<float> output;
    for (const auto& input : stepInput)
    {
        output.push_back(filter.addValue(input));
    }

    // Should be monotonically increasing and asymptotically approach 1.0
    EXPECT_GT(output[0], 0.0f);
    EXPECT_LT(output[0], 1.0f);

    // First sample should be exactly 0.2 (0.2 * 1.0 + 0.8 * 0.0)
    EXPECT_FLOAT_EQ(output[0], 0.2f);

    for (std::size_t i = 1; i < output.size(); ++i)
    {
        EXPECT_GE(output[i], output[i - 1]);  // Monotonically increasing
    }

    // With α=0.2, after 50 iterations should be very close to 1.0
    // Theoretical: 1 - (0.8)^50 ≈ 0.9999
    EXPECT_TRUE(TestLowPassEMA::isApproxEqual(output.back(), 1.0f, 0.001f));

    // Check that we're getting closer to 1.0 over time
    EXPECT_GT(output.back(), output[output.size() / 2]);  // Later values > middle values
}

TEST(UITE_ROVER_LIB2_LowPassEMA, AlphaEffects)
{
    // Test different alpha values with same input
    Filters::LowPassEMA fastFilter(0.8f, 0.0f);  // Fast response
    Filters::LowPassEMA slowFilter(0.1f, 0.0f);  // Slow response

    const float input = 10.0f;

    const float fastResult = fastFilter.addValue(input);
    const float slowResult = slowFilter.addValue(input);

    // Fast filter should respond more quickly
    EXPECT_GT(fastResult, slowResult);

    // Fast filter: 0.8 * 10 + 0.2 * 0 = 8
    EXPECT_FLOAT_EQ(fastResult, 8.0f);

    // Slow filter: 0.1 * 10 + 0.9 * 0 = 1
    EXPECT_FLOAT_EQ(slowResult, 1.0f);
}

TEST(UITE_ROVER_LIB2_LowPassEMA, EdgeCaseAlphaValues)
{
    // Alpha = 0: Should act like a memory (no new input accepted)
    Filters::LowPassEMA noResponseFilter(0.0f, 5.0f);
    const float result1 = noResponseFilter.addValue(100.0f);
    EXPECT_FLOAT_EQ(result1, 5.0f);  // Should remain at initial value

    // Alpha = 1: Should act like direct pass-through
    Filters::LowPassEMA passThroughFilter(1.0f, 5.0f);
    const float result2 = passThroughFilter.addValue(100.0f);
    EXPECT_FLOAT_EQ(result2, 100.0f);  // Should jump to new value immediately
}

TEST(UITE_ROVER_LIB2_LowPassEMA, Reset)
{
    Filters::LowPassEMA filter(0.5f, 0.0f);

    // Add some values
    filter.addValue(10.0f);
    filter.addValue(20.0f);
    EXPECT_NE(filter.getFilteredValue(), 0.0f);

    // Reset and verify
    filter.reset(0.0f);
    EXPECT_FLOAT_EQ(filter.getFilteredValue(), 0.0f);

    // Reset to different value
    filter.reset(42.0f);
    EXPECT_FLOAT_EQ(filter.getFilteredValue(), 42.0f);

    // Verify filter still works after reset
    const float result = filter.addValue(0.0f);
    EXPECT_FLOAT_EQ(result, 21.0f);  // 0.5 * 0 + 0.5 * 42 = 21
}

TEST(UITE_ROVER_LIB2_LowPassEMA, NoisySignalFiltering)
{
    Filters::LowPassEMA filter(0.3f, 5.0f);  // Start near the signal mean to avoid transient effects

    // Create noisy signal around value 5.0
    const std::vector<float> noisyInput = {5.2f, 4.8f, 5.1f, 4.9f, 5.0f, 5.3f, 4.7f, 5.1f, 5.0f, 4.9f, 5.2f, 4.8f};
    std::vector<float> output;

    for (const auto& input : noisyInput)
    {
        output.push_back(filter.addValue(input));
    }

    // Calculate variation using standard deviation approach
    const float inputMean = 5.0f;
    float outputMean = 0.0f;

    for (const auto& val : output)
    {
        outputMean += val;
    }
    outputMean /= static_cast<float>(output.size());

    float inputVariance = 0.0f;
    float outputVariance = 0.0f;

    for (std::size_t i = 0; i < noisyInput.size(); ++i)
    {
        inputVariance += std::pow(noisyInput[i] - inputMean, 2);
        outputVariance += std::pow(output[i] - outputMean, 2);
    }

    inputVariance /= static_cast<float>(noisyInput.size());
    outputVariance /= static_cast<float>(output.size());

    // Output should have less variance (smoother) than input
    EXPECT_LT(outputVariance, inputVariance);

    // Additional check: output should have smaller range than input
    const auto inputMinMax = std::minmax_element(noisyInput.begin(), noisyInput.end());
    const auto outputMinMax = std::minmax_element(output.begin(), output.end());

    const float inputRange = *inputMinMax.second - *inputMinMax.first;
    const float outputRange = *outputMinMax.second - *outputMinMax.first;

    EXPECT_LT(outputRange, inputRange);
}

TEST(UITE_ROVER_LIB2_LowPassEMA, DefaultAlphaValue)
{
    // Test default alpha value (0.6) behavior
    Filters::LowPassEMA filter;  // Uses default alpha = 0.6f

    const float result1 = filter.addValue(10.0f);
    EXPECT_FLOAT_EQ(result1, 6.0f);  // 0.6 * 10 + 0.4 * 0 = 6

    const float result2 = filter.addValue(10.0f);
    EXPECT_FLOAT_EQ(result2, 8.4f);  // 0.6 * 10 + 0.4 * 6 = 8.4
}

TEST(UITE_ROVER_LIB2_LowPassEMA, ConsistentBehavior)
{
    Filters::LowPassEMA filter(0.4f, 2.0f);

    // Multiple calls with same input should converge to that input
    const float target = 8.0f;
    float result = 0.0f;

    constexpr int maxIterations = 100;
    for (int i = 0; i < maxIterations; ++i)
    {
        result = filter.addValue(target);
    }

    // After many iterations, should be very close to target
    EXPECT_TRUE(TestLowPassEMA::isApproxEqual(result, target, 0.01f));
}

TEST(UITE_ROVER_LIB2_LowPassEMA, NegativeValues)
{
    Filters::LowPassEMA filter(0.5f, 0.0f);

    const float result1 = filter.addValue(-10.0f);
    EXPECT_FLOAT_EQ(result1, -5.0f);

    const float result2 = filter.addValue(-10.0f);
    EXPECT_FLOAT_EQ(result2, -7.5f);

    // Test with negative initial value
    Filters::LowPassEMA filter2(0.2f, -5.0f);
    const float result3 = filter2.addValue(10.0f);
    EXPECT_FLOAT_EQ(result3, -2.0f);  // 0.2 * 10 + 0.8 * (-5) = 2 - 4 = -2
}

TEST(UITE_ROVER_LIB2_LowPassEMA, CommonAlphaValues)
{
    // Test behavior with common alpha values from documentation

    // Heavy filtering for noisy sensors (0.1-0.3)
    Filters::LowPassEMA heavyFilter(0.2f, 0.0f);
    const float heavyResult = heavyFilter.addValue(10.0f);
    EXPECT_FLOAT_EQ(heavyResult, 2.0f);  // 0.2 * 10 + 0.8 * 0 = 2

    // Moderate filtering for control loops (0.5-0.7)
    Filters::LowPassEMA moderateFilter(0.6f, 0.0f);
    const float moderateResult = moderateFilter.addValue(10.0f);
    EXPECT_FLOAT_EQ(moderateResult, 6.0f);  // 0.6 * 10 + 0.4 * 0 = 6

    // Light filtering, fast tracking (0.8-0.95)
    Filters::LowPassEMA lightFilter(0.9f, 0.0f);
    const float lightResult = lightFilter.addValue(10.0f);
    EXPECT_FLOAT_EQ(lightResult, 9.0f);  // 0.9 * 10 + 0.1 * 0 = 9

    // Verify ordering: heavy < moderate < light response
    EXPECT_LT(heavyResult, moderateResult);
    EXPECT_LT(moderateResult, lightResult);
}
