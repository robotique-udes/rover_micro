#include "rover_micro_ros_lib/publisher.hpp"

namespace RoverMicroRosLib
{
    Publisher::Publisher(const rosidl_message_type_support_t *msgStruct_, const char *topicName_)
    {
        ASSERT(msgStruct_ == NULL);
        _msgStruct = msgStruct_;

        ASSERT(topicName_ == NULL);
        _topicName = topicName_;
    }
    Publisher::~Publisher(void) {}

    bool Publisher::create(rcl_node_t *nodePtr_)
    {
        RCLC_RET_ON_ERR(rclc_publisher_init_default(&_pub, nodePtr_, _msgStruct, _topicName));
        _alive = true;
        return true;
    }

    void Publisher::destroy(rcl_node_t *nodePtr_)
    {
        _alive = false;
        REMOVE_WARN_UNUSED(rcl_publisher_fini(&_pub, nodePtr_));
    }

    void Publisher::publish(const void *msg)
    {
        if (_alive)
        {
            REMOVE_WARN_UNUSED(rcl_publish(&_pub, msg, NULL));
        }
    }
}
