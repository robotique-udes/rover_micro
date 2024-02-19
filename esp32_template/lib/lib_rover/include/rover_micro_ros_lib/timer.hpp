#ifndef __TIMER_HPP__
#define __TIMER_HPP__

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "helpers/helpers.hpp"

namespace RoverMicroRosLib
{
    class Timer
    {
    public:
        Timer(const uint64_t interval_, const rcl_timer_callback_t callbackFunc_);
        ~Timer(void);
        bool create(rclc_support_t *support_, rclc_executor_t *executor_);
        void destroy(void);

    private:
        rcl_timer_t _timer;
        const rcl_timer_callback_t _callbackFunc;
        const uint64_t _timeoutNs;
    };
}

#endif // __TIMER_HPP__
