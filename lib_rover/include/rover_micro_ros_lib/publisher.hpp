#ifndef __PUBLISHER_HPP__
#define __PUBLISHER_HPP__

#include <rclc/rclc.h>
#include <rclc/executor.h>

#include "helpers/helpers.hpp"

namespace RoverMicroRosLib
{
    class Publisher
    {
    public:
        Publisher(const rosidl_message_type_support_t *msgStruct_, const char *topicName_);
        ~Publisher(void);
        bool create(rcl_node_t *nodePtr_);
        void destroy(rcl_node_t *nodePtr_);
        void publish(const void *msg);

    private:
        rcl_publisher_t _pub;
        const rosidl_message_type_support_t *_msgStruct;
        const char *_topicName;
    };
}

#endif // __PUBLISHER_HPP__
