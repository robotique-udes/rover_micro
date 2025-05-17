#ifndef CIRCULAR_MOVING_AVERAGE_HPP
#define CIRCULAR_MOVING_AVERAGE_HPP

#include <cmath>
#include <array>
#include <Arduino.h>

// On pourrait potentiellement l'update pour gerer des rads

/**
 * @brief 
 * Circular moving average filter for angular data (e.g. compass heading or encoder values).
 * Smooths transitions around the wrap-around point (e.g., 359° to 0°).
 * @attention Works only for 0-360 degrees range
 * @tparam WINDOW_SIZE The number of samples to average over
 */
template<uint16_t WINDOW_SIZE>
class CircularMovingAverage
{
  private:
    std::array<float, WINDOW_SIZE> _buffer = {0.0f};
    uint16_t _index = 0;
    uint16_t _count = 0;
    float _sinSum = 0.0f;
    float _cosSum = 0.0f;

  public:
    CircularMovingAverage() {}
    ~CircularMovingAverage() {}

    float addValue(float headingDeg_)
    {
        float newRad = headingDeg_ * DEG_TO_RAD;
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

        _buffer[_index] = newRad;
        _index = (_index + 1) % WINDOW_SIZE;

        _sinSum += std::sin(newRad);
        _cosSum += std::cos(newRad);

        return getAverage();
    }

    float getAverage(void) const
    {
        if (_count == 0)
        {
            return 0.0f;
        }
        float avgRad = std::atan2(_sinSum / _count, _cosSum / _count);
        if (avgRad < 0)
        {
            avgRad += 2.0f * PI;
        }
        return avgRad * RAD_TO_DEG;
    }
};

#endif  // CIRCULAR_MOVING_AVERAGE_HPP
