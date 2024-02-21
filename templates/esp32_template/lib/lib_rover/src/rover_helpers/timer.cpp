#ifndef __TIMER_CPP__
#define __TIMER_CPP__

#include "rover_helpers/timer.hpp"

namespace RoverHelpers
{
    template <typename TYPE, TYPE (*CLOCK_FUNC)(void)>
    Timer<TYPE, CLOCK_FUNC>::
        Timer()
    {
        _interval = 0;
        _prevClock = 0;
    }

    template <typename TYPE, TYPE (*CLOCK_FUNC)(void)>
    Timer<TYPE, CLOCK_FUNC>::
        Timer(TYPE interval_)
    {
        _interval = 0;
        _prevClock = 0;

        this->init(interval_);
    }

    template <typename TYPE, TYPE (*CLOCK_FUNC)(void)>
    Timer<TYPE, CLOCK_FUNC>::
        ~Timer() {}

    template <typename TYPE, TYPE (*CLOCK_FUNC)(void)>
    void Timer<TYPE, CLOCK_FUNC>::
        init(TYPE interval_)
    {
        setInterval(interval_);
        _prevClock = CLOCK_FUNC();
    }

    template <typename TYPE, TYPE (*CLOCK_FUNC)(void)>
    bool Timer<TYPE, CLOCK_FUNC>::
        isDone(bool reset_)
    {
        if ((_prevClock + _interval) < CLOCK_FUNC())
        {
            if (reset_)
            {
                _prevClock = CLOCK_FUNC() - 1;
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    template <typename TYPE, TYPE (*CLOCK_FUNC)(void)>
    void Timer<TYPE, CLOCK_FUNC>::
        updateInterval(TYPE newInterval_)
    {
        _interval = newInterval_;
    }

    template <typename TYPE, TYPE (*CLOCK_FUNC)(void)>
    void Timer<TYPE, CLOCK_FUNC>::
        reset(void)
    {
        _prevClock = CLOCK_FUNC();
    }

    template <typename TYPE, TYPE (*CLOCK_FUNC)(void)>
    void Timer<TYPE, CLOCK_FUNC>::
        setInterval(TYPE interval_)
    {
        _interval = interval_;
    }
}

#endif
