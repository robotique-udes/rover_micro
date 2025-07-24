#ifndef ROVER_LIB2_FILTERS_CIRCULAR_MOVING_AVERAGE_HPP
#define ROVER_LIB2_FILTERS_CIRCULAR_MOVING_AVERAGE_HPP

#include <array>

/**
 * @brief
 * Circular moving average filter for angular data in radians (e.g., compass heading or encoder values).
 * Smooths transitions around the wrap-around point (e.g., near 2π to 0).
 * @attention Works only for values wrapped in the range [0, 2π)
 * @tparam WINDOW_SIZE The number of samples to average over
 */
template<size_t WINDOW_SIZE>
class CircularMovingAverage
{
  private:
    std::array<float, WINDOW_SIZE> _buffer = {0.0F};
    uint16_t _index = 0U;
    uint16_t _count = 0U;
    float _sinSum = 0.0F;
    float _cosSum = 0.0F;

  public:
    float addValue(float angleRad_)
    {
        float oldRad = _buffer[_index];

        if (_count < WINDOW_SIZE)
        {
            _count++;
        }
        else
        {
            _sinSum -= std::sin(oldRad);
            _cosSum -= std::cos(oldRad);
        }

        _buffer[_index] = angleRad_;
        _index = (++_index >= WINDOW_SIZE) ? 0 : _index;

        _sinSum += std::sin(angleRad_);
        _cosSum += std::cos(angleRad_);

        return getAverage();
    }

    float getAverage(void) const
    {
        if (_count == 0U)
        {
            return 0.0F;
        }

        float avgRad = std::atan2(_sinSum / _count, _cosSum / _count);
        if (avgRad < 0.0F)
        {
            avgRad += 2.0F * std::numbers::pi_v<float>;
        }
        return avgRad;
    }
};

#endif  // ROVER_LIB2_FILTERS_CIRCULAR_MOVING_AVERAGE_HPP
