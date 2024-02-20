#ifndef __MOVING_AVERAGE_HPP__
#define __MOVING_AVERAGE_HPP__

// =============================================================================
// moving_average.hpp defines a class for making moving_average / lowpass filter
// easier in code.
// =============================================================================

#include "rover_helpers/helpers.hpp"

namespace RoverHelpers
{
    /// @brief MovingAverage class wrapper with fixed number of coefficients.
    /// @tparam TYPE The type of data stored into the MovingAverage, it's the user
    /// job to make sure the type makes sense for a MovingAverage.
    /// @tparam COEFF_NB The number of elements contained into the moving average.
    template <typename TYPE, uint16_t COEFF_NB>
    class MovingAverage
    {
    public:
        /// @brief MovingAverage's constructor.
        /// @param startingValue_ The average will start at this specific value,
        /// default is 0.
        MovingAverage(TYPE startingValue_ = static_cast<TYPE>(0));

        /// @brief MovingAverage's destructor
        ~MovingAverage();

        /// @brief Adds a value to the movingAverage.
        /// @param value_ value to add to the average.
        /// @return The new average
        float addValue(TYPE value_);

        /// @return Current average.
        float getAverage(void);

    private:
        TYPE _avgTable[COEFF_NB] = {0};
        uint16_t _cursor = 0;
        float _avg = 0.0f;
    };
}

#include "rover_helpers/moving_average.cpp"

#endif // __MOVING_AVERAGE_HPP__
