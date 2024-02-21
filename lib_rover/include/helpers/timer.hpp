#ifndef __TIMER_H__
#define __TIMER_H__

#include "Arduino.h"

template <typename TYPE, TYPE (*clockFunc)(void)>
class Timer
{
private:
    TYPE _prevClock;
    TYPE _interval;

    void setInterval(TYPE interval)
    {
        _interval = interval;
    }

public:
    Timer()
    {
        _interval = 0;
        _prevClock = 0;
    }
    Timer(TYPE interval_)
    {
        _interval = 0;
        _prevClock = 0;

        this->init(interval_);
    }
    ~Timer() {}

    void init(TYPE interval)
    {
        setInterval(interval);
        _prevClock = clockFunc();
    }

    bool isDone(bool reset = 1)
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

    void updateInterval(TYPE newInterval_)
    {
        _interval = newInterval_;
    }

    void reset(void)
    {
        _prevClock = clockFunc();
    }
};

template <typename TYPE, TYPE (*clockFunc)(void)>
class Chrono
{
private:
    TYPE _startClock;
    TYPE _accumulatedTime;
    bool _paused;

public:
    Chrono()
    {
        _paused = false;
        _accumulatedTime = 0;
        _startClock = 0;
        this->init();
    }
    ~Chrono() {}

    void init()
    {
        _accumulatedTime = 0;
        _startClock = clockFunc();
    }

    TYPE pause(void)
    {
        if (!_paused)
        {
            _accumulatedTime = this->getTime();
            _paused = true;
        }
    }

    void resume(void)
    {
        if (_paused)
        {
            _startClock = clockFunc();
            _paused = false;
        }
    }

    TYPE getTime(void)
    {
        if (_paused)
        {
            return _accumulatedTime;
        }
        else
        {
            return _accumulatedTime + clockFunc() - _startClock;
        }
    }

    void restart(void)
    {
        this->init();
    }
};

class TimerMillis
{
private:
    unsigned long _prevMillis;
    unsigned long _interval;

    void setInterval(unsigned long interval)
    {
        _interval = interval;
    }

public:
    TimerMillis()
    {
        _interval = 0;
        _prevMillis = 0;
    }
    TimerMillis(unsigned long interval)
    {
        _interval = 0;
        _prevMillis = 0;
        this->init(interval);
    }
    ~TimerMillis() {}

    void init(unsigned long interval)
    {
        setInterval(interval);
        _prevMillis = millis();
    }

    bool done(bool reset = 1)
    {
        if ((_prevMillis + _interval) < millis())
        {
            if (reset)
            {
                _prevMillis = millis() - 1;
            }
            return true;
        }
        else
        {
            return false;
        }
    }
};

class TimerMicros
{
private:
    unsigned long _prevMicros;
    unsigned long _interval;

    void setInterval(unsigned long interval)
    {
        _interval = interval;
    }

public:
    TimerMicros()
    {
        _interval = 0;
        _prevMicros = 0;
    }
    TimerMicros(unsigned long interval)
    {
        _interval = 0;
        _prevMicros = 0;
        this->init(interval);
    }
    ~TimerMicros() {}

    void init(unsigned long interval)
    {
        setInterval(interval);
        _prevMicros = micros();
    }

    bool done(bool reset = 1)
    {
        if ((_prevMicros + _interval) < micros())
        {
            if (reset)
            {
                _prevMicros = micros() - 1;
            }
            return true;
        }
        else
        {
            return false;
        }
    }
};

#endif //__TIMER_H__
