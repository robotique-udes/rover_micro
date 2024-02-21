#ifndef __SUBSCRIBER_HPP__
#define __SUBSCRIBER_HPP__

#if !defined (ESP32)
#error CPU not supported
#endif

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "rover_helpers/helpers.hpp"

namespace RoverMicroRosLib
{
    class Subscriber
    {
    public:
        Subscriber(const rosidl_message_type_support_t *msgTypeStruct_,
                   const char *topicName_,
                   const rclc_subscription_callback_t callbackFunc_,
                   const rclc_executor_handle_invocation_t invocation_ = rclc_executor_handle_invocation_t::ON_NEW_DATA);
        ~Subscriber(void);
        bool create(rcl_node_t *node_, rclc_executor_t *executor_);
        void destroy(rcl_node_t *node_);

    private:
        rcl_subscription_t _sub;
        const rosidl_message_type_support_t *_msgTypeStruct;
        const char *_topicName;
        const rclc_subscription_callback_t _callbackFunc;
        const rclc_executor_handle_invocation_t _invocation;
    };
}

#endif // __SUBSCRIBER_HPP__
