#ifndef ROVER_LIB2_FILTERS_MOVING_AVERAGE_HPP
#define ROVER_LIB2_FILTERS_MOVING_AVERAGE_HPP

#include "rover_lib2/filters/filter.hpp"
#include "rover_lib2/helpers/macros.hpp"

#include <cstdint>

namespace Filters
{
    template<class T, uint16_t COEFF_NB>
    class MovingAverage
    {
      public:
        MovingAverage(T startingValue = 0)
        {
            reset(static_cast<float>(startingValue));
        }

        ~MovingAverage() = default;

        float addValue(T value)
        {
            _avg -= static_cast<float>(_avgTable[_cursor]);
            _avg += static_cast<float>(value);
            _avgTable[_cursor] = value;
            _cursor = (_cursor + 1) % COEFF_NB;
            return getFilteredValue();
        }

        float getFilteredValue() const
        {
            return _avg / static_cast<float>(COEFF_NB);
        }

        void reset(float fillValue_)
        {
            _avg = fillValue_ * COEFF_NB;
            for (uint16_t i = 0; i < COEFF_NB; ++i)
            {
                _avgTable[i] = static_cast<T>(fillValue_);
            }
            _cursor = 0;
        }

      private:
        T _avgTable[COEFF_NB] = {};
        uint16_t _cursor = 0;
        float _avg = 0.0f;

        VALIDATE_CONCEPT(Filter, MovingAverage);
    };
}  // namespace Filters

#endif  // ROVER_LIB2_FILTERS_MOVING_AVERAGE_HPP
