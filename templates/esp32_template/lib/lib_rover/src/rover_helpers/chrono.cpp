#ifndef __CHRONO_CPP__
#define __CHRONO_CPP__

#include "rover_helpers/chrono.hpp"

namespace RoverHelpers
{
    template <typename TYPE, TYPE (*CLOCK_FUNC)(void)>
    Chrono<TYPE, CLOCK_FUNC>::
        Chrono()
    {
        _paused = false;
        _accumulatedTime = 0;
        _startClock = 0;
        this->init();
    }

    template <typename TYPE, TYPE (*CLOCK_FUNC)(void)>
    Chrono<TYPE, CLOCK_FUNC>::
        ~Chrono()
    {
    }

    template <typename TYPE, TYPE (*CLOCK_FUNC)(void)>
    void Chrono<TYPE, CLOCK_FUNC>::
        init()
    {
        _accumulatedTime = 0;
        _startClock = CLOCK_FUNC();
    }

    template <typename TYPE, TYPE (*CLOCK_FUNC)(void)>
    TYPE Chrono<TYPE, CLOCK_FUNC>::
        pause(void)
    {
        if (!_paused)
        {
            _accumulatedTime = this->getTime();
            _paused = true;
        }
    }

    template <typename TYPE, TYPE (*CLOCK_FUNC)(void)>
    void Chrono<TYPE, CLOCK_FUNC>::
        resume(void)
    {
        if (_paused)
        {
            _startClock = CLOCK_FUNC();
            _paused = false;
        }
    }

    template <typename TYPE, TYPE (*CLOCK_FUNC)(void)>
    TYPE Chrono<TYPE, CLOCK_FUNC>::
        getTime(void)
    {
        if (_paused)
        {
            return _accumulatedTime;
        }
        else
        {
            return _accumulatedTime + CLOCK_FUNC() - _startClock;
        }
    }

    template <typename TYPE, TYPE (*CLOCK_FUNC)(void)>
    void Chrono<TYPE, CLOCK_FUNC>::
        restart(void)
    {
        this->init();
    }

#include "rover_helpers/chrono.cpp"
}

#endif // __CHRONO_CPP__
