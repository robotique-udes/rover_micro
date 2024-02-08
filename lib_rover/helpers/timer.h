#ifndef __TIMER_H__
#define __TIMER_H__

#include "Arduino.h"

template <typename T, unsigned long (*clockFunc)(void)>
class Timer
{
private:
    T _prevClock;
    T _interval;

    void setInterval(T interval)
    {
        _interval = interval;
    }

public:
    Timer()
    {
        _interval = 0;
        _prevClock = 0;
    }
    Timer(T interval)
    {
        _interval = 0;
        _prevClock = 0;
        this->init(interval);
    }
    ~Timer() {}

    void init(T interval)
    {
        setInterval(interval);
        _prevClock = clockFunc();
    }

    bool isDone(bool reset = 1)
    {
        if ((_prevClock + _interval) <= clockFunc())
        {
            if (reset)
            {
                _prevClock = clockFunc();
            }
            return true;
        }
        else
        {
            return false;
        }
    }

    void reset()
    {
        _prevClock = clockFunc();
    }
};

template <typename T, T (*clockFunc)(void)>
class Chrono
{
public:
    Chrono()
    {
        this->start();
    }
    ~Chrono() {}

    void start()
    {
        startTime = clockFunc();
    }

    T getTime()
    {
        return clockFunc() - startTime;
    }

private:
    T startTime;
};

#endif //__TIMER_H__
