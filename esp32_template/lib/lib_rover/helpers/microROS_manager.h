#ifndef __MICRO_ROS_MANAGER_H__
#define __MICRO_ROS_MANAGER_H__

#include "Arduino.h"
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>

#include "helpers/logger.h"
#include "helpers/timer.h"
#include "helpers/macros.h"

class MicroRosPublisher;

template <uint8_t NB_PUBLISHER, uint8_t NB_SUBSCRIBER>
class MicroROSManager
{
protected:
    enum eConnectionStates : uint8_t
    {
        WAITING_AGENT,
        AGENT_AVAILABLE,
        AGENT_CONNECTED,
        AGENT_DISCONNECTED
    };

    eConnectionStates _connectionState = eConnectionStates::WAITING_AGENT;
    TimerMillis _timerCheckReconnect;
    TimerMillis _timerCheckDisconnect;
    bool _withLED;
    uint8_t _ledPIN;

public:
    MicroROSManager(const char *namespaceName_,
                    const char *nodeName_,
                    bool withLogger_ = true,
                    bool withLED_ = true,
                    uint8_t ledPIN_ = LED_BUILTIN,
                    uint32_t connectionValidationInterval_ = 200UL,
                    uint32_t reconnectionInterval_ = 500UL)
    {
        ASSERT(namespaceName_ == NULL);
        _namespaceName = namespaceName_;

        ASSERT(nodeName_ == NULL);
        _nodeName = nodeName_;

        _withLogger = withLogger_;
        _withLED = withLED_;
        _ledPIN = ledPIN_;

        TimerMillis _timerCheckDisconnect = TimerMillis(connectionValidationInterval_);
        TimerMillis _timerCheckReconnect = TimerMillis(reconnectionInterval_);

        for (uint8_t i = 0; i < NB_PUBLISHER; i++)
        {
            _pubs[i] = NULL;
        }
    }
    ~MicroROSManager() {}
    void init()
    {
        if (_withLED)
        {
            pinMode(_ledPIN, OUTPUT);
        }
    }
    void addPublisher(MicroRosPublisher *pub_)
    {
        ASSERT(pub_ == NULL)
        _pubs[_pubCounter] = pub_;
        _pubCounter++;
    }
    void spinSome(uint32_t timeout_mS = 0UL)
    {
        switch (_connectionState)
        {
        case WAITING_AGENT:
            if (_timerCheckReconnect.done())
            {
                _connectionState = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;
            }
            break;
        case AGENT_AVAILABLE:
            _connectionState = (true == this->createEntities()) ? AGENT_CONNECTED : WAITING_AGENT;
            if (_connectionState == WAITING_AGENT)
            {
                this->destroyEntities();
            };
            break;
        case AGENT_CONNECTED:
            if (_timerCheckDisconnect.done())
            {
                _connectionState = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            }

            if (_connectionState == AGENT_CONNECTED)
            {
                rclc_executor_spin_some(&executor, RCL_MS_TO_NS(timeout_mS));
            }
            break;
        case AGENT_DISCONNECTED:
            this->destroyEntities();
            _connectionState = WAITING_AGENT;
            break;
        default:
            break;
        }

        this->updateLED();
    }
    uint8_t getConnectionState()
    {
        return _connectionState;
    }
    void updateLED()
    {
        if (_withLED)
        {
            if (_connectionState == AGENT_CONNECTED)
            {
                digitalWrite(_ledPIN, HIGH);
            }
            else
            {
                digitalWrite(_ledPIN, LOW);
            }
        }
    }

private:
    rclc_support_t support;
    rcl_node_t node;
    rcl_timer_t timer;
    rclc_executor_t executor;
    rcl_allocator_t allocator;
    const char *_nodeName = NULL;
    const char *_namespaceName = NULL;
    MicroRosPublisher *_pubs[NB_PUBLISHER];
    uint8_t _pubCounter = 0;
    bool _withLogger;

    bool createEntities(void)
    {
        allocator = rcl_get_default_allocator();
        RCLC_RET_ON_ERR(rclc_support_init(&support, 0, NULL, &allocator));
        RCLC_RET_ON_ERR(rclc_node_init_default(&node, _nodeName, _namespaceName, &support));

        if (_withLogger)
        {
            Logger.createLogger(&node, _nodeName, _namespaceName);
        }

        if (_pubCounter + 1 > NB_PUBLISHER)
        {
            LOG(ERROR, "Maximum number of publisher already created");
            delay(2000);
        }

        for (uint8_t i = 0; i < _pubCounter; i++)
        {
            if (_pubs[i] != NULL)
            {
                _pubs[i]->create(&node);
            }
        }

        // RCLC_RET_ON_ERR(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(1000), cbTimer));
        // RCLC_RET_ON_ERR(rclc_subscription_init_default(&sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "sub_topic_name"))

        executor = rclc_executor_get_zero_initialized_executor();
        RCLC_RET_ON_ERR(rclc_executor_init(&executor, &support.context, 1, &allocator));
        // RCLC_RET_ON_ERR(rclc_executor_add_timer(&executor, &timer));

        // ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32) msg;
        // RCLC_RET_ON_ERR(rclc_executor_add_subscription(&executor, &sub, &msg, &cbSubscriber, ON_NEW_DATA));

        return true;
    }
    void destroyEntities(void)
    {
        rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
        (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

        // "Fini" each element created inside the createEntities
        if (_withLogger)
        {
            Logger.destroyLogger(&node);
        }

        for (uint8_t i = 0; i < _pubCounter; i++)
        {
            if (_pubs[i] != NULL)
            {
                _pubs[i]->destroy(&node);
            }
        }

        // REMOVE_WARN_UNUSED(rcl_timer_fini(&timer));
        // REMOVE_WARN_UNUSED(rcl_subscription_fini(&sub, &node));
        REMOVE_WARN_UNUSED(rclc_executor_fini(&executor));
        REMOVE_WARN_UNUSED(rcl_node_fini(&node));
        REMOVE_WARN_UNUSED(rclc_support_fini(&support));
    }
};

class MicroRosPublisher
{
private:
    rcl_publisher_t _pub;
    const rosidl_message_type_support_t *_msgStruct;
    const char *_topicName;

public:
    MicroRosPublisher(const rosidl_message_type_support_t *msgStruct_, const char *topicName_)
    {
        ASSERT(msgStruct_ == NULL);
        _msgStruct = msgStruct_;

        ASSERT(topicName_ == NULL);
        _topicName = topicName_;
    }
    ~MicroRosPublisher() {}

    bool create(rcl_node_t *nodePtr_)
    {
        RCLC_RET_ON_ERR(rclc_publisher_init_default(&_pub, nodePtr_, _msgStruct, _topicName));
        return true;
    }

    void destroy(rcl_node_t *nodePtr_)
    {
        REMOVE_WARN_UNUSED(rcl_publisher_fini(&_pub, nodePtr_));
    }

    void publish(const void *msg)
    {
        rcl_publish(&_pub, msg, NULL);
    }
};

#endif // __MICRO_ROS_MANAGER_H__
