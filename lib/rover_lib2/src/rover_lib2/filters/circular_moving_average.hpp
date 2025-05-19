#ifndef CIRCULAR_MOVING_AVERAGE_HPP
#define CIRCULAR_MOVING_AVERAGE_HPP

#include <cmath>
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
    std::array<float, WINDOW_SIZE> _buffer = {0.0f};
    uint16_t _index = 0;
    uint16_t _count = 0;
    float _sinSum = 0.0f;
    float _cosSum = 0.0f;

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
        if (_count == 0)
        {
            return 0.0f;
        }

        float avgRad = std::atan2(_sinSum / _count, _cosSum / _count);
        if (avgRad < 0.0f)
        {
            avgRad += 2.0f * static_cast<float>(M_PI);
        }
        return avgRad;
    }
};

#endif  // CIRCULAR_MOVING_AVERAGE_HPP
