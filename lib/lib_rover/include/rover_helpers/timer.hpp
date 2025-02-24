#ifndef __TIMER_H__
#define __TIMER_H__

namespace RoverHelpers
{
    /// @brief Timer helper class. A wrapper to do block of code after a certain
    /// time or at a certain frequency.
    /// @tparam TYPE The return type of the clock function used in 2nd template
    /// argument
    /// @tparam CLOCK_FUNC A function that returns the current clock of the
    /// processor. In arduino framework: millis or micros
    /// @example Executing a block of code at 1 Hz (each 1000 milliseconds)
    /// void setup()
    /// {
    ///     RoverHelpers::Timer<unsigned long, millis> timer(1000);
    ///
    ///     for (EVER)
    ///     {
    ///         if (timer.isDone())
    ///         {
    ///             /* Do something at 1 Hz*/
    ///         }
    ///     }
    /// }
    template <typename TYPE, TYPE (*clockFunc)(void)>
    class Timer
    {
    private:
        TYPE _prevClock;
        TYPE _interval;

        void setInterval(TYPE interval);

    public:
        Timer(void);
        Timer(TYPE interval_);
        ~Timer(void);
        void init(TYPE interval_);
        bool isDone(bool reset_ = 1);
        void updateInterval(TYPE newInterval_);
        TYPE getInterval(void);
        void reset(void);
    };
}

#include "rover_helpers/timer.cpp"

#endif //__TIMER_H__
