#include "rover_micro_ros_lib/subscriber.hpp"

namespace RoverMicroRosLib
{
    Subscriber::Subscriber(const rosidl_message_type_support_t *msgTypeStruct_,
                           const char *topicName_,
                           const rclc_subscription_callback_t callbackFunc_,
                           const rclc_executor_handle_invocation_t invocation_)
        : _msgTypeStruct(msgTypeStruct_),
          _topicName(topicName_),
          _callbackFunc(callbackFunc_),
          _invocation(invocation_)
    {
    }

    Subscriber::~Subscriber(void)
    {
    }

    bool Subscriber::create(rcl_node_t *node_, rclc_executor_t *executor_)
    {
        RCLC_RET_ON_ERR(rclc_subscription_init_default(&_sub, node_, _msgTypeStruct, _topicName));
        RCLC_RET_ON_ERR(rclc_executor_add_subscription(executor_,
                                                       &_sub,
                                                       (void *)_msgTypeStruct,
                                                       _callbackFunc,
                                                       _invocation));
        return true;
    }

    void Subscriber::destroy(rcl_node_t *node_)
    {
        REMOVE_WARN_UNUSED(rcl_subscription_fini(&_sub, node_));
    }
}
