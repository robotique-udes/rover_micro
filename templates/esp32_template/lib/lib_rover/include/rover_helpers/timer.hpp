#ifndef __TIMER_H__
#define __TIMER_H__

namespace RoverHelpers
{
    template <typename T, T (*clockFunc)(void)>
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
        Timer(T interval_)
        {
            _interval = 0;
            _prevClock = 0;

            this->init(interval_);
        }
        ~Timer() {}

        void init(T interval)
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

        void updateInterval(T newInterval_)
        {
            _interval = newInterval_;
        }

        void reset(void)
        {
            _prevClock = clockFunc();
        }
    };

    template <typename T, T (*clockFunc)(void)>
    class Chrono
    {
    private:
        T _startClock;
        T _accumulatedTime;
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

        T pause(void)
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

        T getTime(void)
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
}

#endif //__TIMER_H__
