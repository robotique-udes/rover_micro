#include "rover_micro_ros_lib/timer.hpp"

#if defined(MICRO_ROS_LOGGER)

namespace RoverMicroRosLib
{
    Timer::Timer(const uint64_t interval_, const rcl_timer_callback_t callbackFunc_) : _timeoutNs(interval_),
                                                                                       _callbackFunc(callbackFunc_)
    {
    }

    Timer::~Timer(void)
    {
    }

    bool Timer::create(rclc_support_t *support_, rclc_executor_t *executor_)
    {
        RCLC_RET_ON_ERR(rclc_timer_init_default(&_timer, support_, _timeoutNs, _callbackFunc));
        RCLC_RET_ON_ERR(rclc_executor_add_timer(executor_, &_timer));
        return true;
    }

    void Timer::destroy(void)
    {
        REMOVE_WARN_UNUSED(rcl_timer_fini(&_timer));
    }
}

#endif // defined(MICRO_ROS_LOGGER)
