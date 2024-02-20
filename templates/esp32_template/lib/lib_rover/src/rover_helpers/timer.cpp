#ifndef __TIMER_CPP__
#define __TIMER_CPP__

#include "rover_helpers/timer.hpp"

namespace RoverHelpers
{
    template <typename T, T (*clockFunc)(void)>
    Timer<T, clockFunc>::
        Timer()
    {
        _interval = 0;
        _prevClock = 0;
    }

    template <typename T, T (*clockFunc)(void)>
    Timer<T, clockFunc>::
        Timer(T interval_)
    {
        _interval = 0;
        _prevClock = 0;

        this->init(interval_);
    }

    template <typename T, T (*clockFunc)(void)>
    Timer<T, clockFunc>::
        ~Timer() {}

    template <typename T, T (*clockFunc)(void)>
    void Timer<T, clockFunc>::
        init(T interval)
    {
        setInterval(interval);
        _prevClock = clockFunc();
    }

    template <typename T, T (*clockFunc)(void)>
    bool Timer<T, clockFunc>::
        isDone(bool reset = 1)
    {
        if ((_prevClock + _interval) < clockFunc())
        {
            if (reset)
            {
                _prevClock = clockFunc() - 1;
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    template <typename T, T (*clockFunc)(void)>
    void Timer<T, clockFunc>::
        updateInterval(T newInterval_)
    {
        _interval = newInterval_;
    }

    template <typename T, T (*clockFunc)(void)>
    void Timer<T, clockFunc>::
        reset(void)
    {
        _prevClock = clockFunc();
    }

    template <typename T, T (*clockFunc)(void)>
    void Timer<T, clockFunc>::
        setInterval(T interval)
    {
        _interval = interval;
    }
}

#endif
