#ifndef __MOVING_AVERAGE_H__
#define __MOVING_AVERAGE_H__

#include "helpers/macros.h"

template <class T, uint16_t SIZE>
class MovingAverage
{
private:
    T _avgTable[SIZE] = {0};
    uint16_t _cursor = 0;
    float _avg = 0.0f;

public:
    MovingAverage(T defaultValue_ = static_cast<T>(0.0f))
    {
        for (uint16_t i = 0; i < SIZE; i++)
        {
            this->addValue(defaultValue_);
        }
    }
    ~MovingAverage() {}

    float addValue(T value)
    {
        _avg -= (float)_avgTable[_cursor];
        _avg += (float)value;
        _avgTable[_cursor] = value;
        _cursor = _cursor + 1 == SIZE ? 0 : _cursor + 1;
        return _avg / (float)SIZE;
    }

    float getAverage(void)
    {
        return _avg / static_cast<float>(SIZE);
    }
};

#endif
