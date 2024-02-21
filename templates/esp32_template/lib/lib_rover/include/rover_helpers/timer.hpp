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

        void setInterval(T interval);

    public:
        Timer();
        Timer(T interval_);
        ~Timer();
        void init(T interval);
        bool isDone(bool reset = 1);
        void updateInterval(T newInterval_);
        void reset(void);
    };
}

#include "rover_helpers/timer.cpp"

#endif //__TIMER_H__
