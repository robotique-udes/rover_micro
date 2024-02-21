#include "rover_micro_ros_lib/node.hpp"

#ifndef __NODE_CPP__
#define __NODE_CPP__

namespace RoverMicroRosLib
{
    template <uint8_t NB_PUBLISHER, uint8_t NB_TIMER, uint8_t NB_SUBSCRIBER>
    Node<NB_PUBLISHER, NB_TIMER, NB_SUBSCRIBER>::
        Node(const char *namespaceName_,
             const char *nodeName_,
             bool withLogger_,
             bool withLED_,
             uint8_t ledPIN_,
             uint32_t connectionValidationInterval_,
             uint32_t reconnectionInterval_)
    {
        ASSERT(namespaceName_ == NULL);
        _namespaceName = namespaceName_;

        ASSERT(nodeName_ == NULL);
        _nodeName = nodeName_;

        _withLogger = withLogger_;
        _withLED = withLED_;
        _ledPIN = ledPIN_;

        RoverHelpers::Timer<unsigned long, millis> _timerCheckDisconnect = RoverHelpers::Timer<unsigned long, millis>(connectionValidationInterval_);
        RoverHelpers::Timer<unsigned long, millis> _timerCheckReconnect = RoverHelpers::Timer<unsigned long, millis>(reconnectionInterval_);

        for (uint8_t i = 0; i < NB_PUBLISHER; i++)
        {
            _pubs[i] = NULL;
        }
    }

    template <uint8_t NB_PUBLISHER, uint8_t NB_TIMER, uint8_t NB_SUBSCRIBER>
    Node<NB_PUBLISHER, NB_TIMER, NB_SUBSCRIBER>::
        ~Node(void)
    {
    }

    template <uint8_t NB_PUBLISHER, uint8_t NB_TIMER, uint8_t NB_SUBSCRIBER>
    void Node<NB_PUBLISHER, NB_TIMER, NB_SUBSCRIBER>::
        init(void)
    {
        if (_withLED)
        {
            pinMode(_ledPIN, OUTPUT);
        }
    }

    template <uint8_t NB_PUBLISHER, uint8_t NB_TIMER, uint8_t NB_SUBSCRIBER>
    void Node<NB_PUBLISHER, NB_TIMER, NB_SUBSCRIBER>::
        addPublisher(Publisher *pub_)
    {
        ASSERT(pub_ == NULL)
        _pubs[_pubCounter++] = pub_;
    }

    template <uint8_t NB_PUBLISHER, uint8_t NB_TIMER, uint8_t NB_SUBSCRIBER>
    void Node<NB_PUBLISHER, NB_TIMER, NB_SUBSCRIBER>::
        addTimer(Timer *timer_)
    {
        ASSERT(timer_ == NULL)
        _timers[_timerCounter++] = timer_;
    }

    template <uint8_t NB_PUBLISHER, uint8_t NB_TIMER, uint8_t NB_SUBSCRIBER>
    void Node<NB_PUBLISHER, NB_TIMER, NB_SUBSCRIBER>::
        addSubscriber(Subscriber *sub_)
    {
        ASSERT(sub_ == NULL)
        _subs[_subCounter++] = sub_;
    }

    template <uint8_t NB_PUBLISHER, uint8_t NB_TIMER, uint8_t NB_SUBSCRIBER>
    void Node<NB_PUBLISHER, NB_TIMER, NB_SUBSCRIBER>::
        spinSome(const uint32_t timeoutNs_)
    {
        switch (_connectionState)
        {
        case WAITING_AGENT:
            if (_timerCheckReconnect.isDone())
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
            if (_timerCheckDisconnect.isDone())
            {
                _connectionState = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;
            }

            if (_connectionState == AGENT_CONNECTED)
            {
                rclc_executor_spin_some(&executor, timeoutNs_);
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

    template <uint8_t NB_PUBLISHER, uint8_t NB_TIMER, uint8_t NB_SUBSCRIBER>
    uint8_t Node<NB_PUBLISHER, NB_TIMER, NB_SUBSCRIBER>::
        getConnectionState(void)
    {
        return _connectionState;
    }

    template <uint8_t NB_PUBLISHER, uint8_t NB_TIMER, uint8_t NB_SUBSCRIBER>
    void Node<NB_PUBLISHER, NB_TIMER, NB_SUBSCRIBER>::updateLED()
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

    template <uint8_t NB_PUBLISHER, uint8_t NB_TIMER, uint8_t NB_SUBSCRIBER>
    bool Node<NB_PUBLISHER, NB_TIMER, NB_SUBSCRIBER>::
        createEntities(void)
    {
        allocator = rcl_get_default_allocator();
        RCLC_RET_ON_ERR(rclc_support_init(&support, 0, NULL, &allocator));

        RCLC_RET_ON_ERR(rclc_node_init_default(&node, _nodeName, _namespaceName, &support));

        executor = rclc_executor_get_zero_initialized_executor();

        size_t numberOfHandle = NB_SUBSCRIBER + NB_TIMER;
        numberOfHandle = numberOfHandle < 1 ? 1 : numberOfHandle; // Make sure there's at least 1 handle
        RCLC_RET_ON_ERR(rclc_executor_init(&executor, &support.context, numberOfHandle, &allocator));

        if (_withLogger)
        {
            G_Logger.createLogger(&node, _nodeName, _namespaceName);
        }

        ASSERT(_pubCounter > NB_PUBLISHER, "Maximum number of publisher already created")
        for (uint8_t i = 0; i < _pubCounter; i++)
        {
            if (_pubs[i] != NULL)
            {
                _pubs[i]->create(&node);
            }
        }

        ASSERT(_timerCounter > NB_TIMER, "Maximum number of timers already created")
        for (uint8_t i = 0; i < _timerCounter; i++)
        {
            if (_timers[i] != NULL)
            {
                _timers[i]->create(&support, &executor);
            }
        }

        ASSERT(_subCounter > NB_SUBSCRIBER, "Maximum number of subscriber already created")
        for (uint8_t i = 0; i < _subCounter; i++)
        {
            if (_subs[i] != NULL)
            {
                _subs[i]->create(&node, &executor);
            }
        }

        return true;
    }

    template <uint8_t NB_PUBLISHER, uint8_t NB_TIMER, uint8_t NB_SUBSCRIBER>
    void Node<NB_PUBLISHER, NB_TIMER, NB_SUBSCRIBER>::
        destroyEntities(void)
    {
        rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
        (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

        if (_withLogger)
        {
            G_Logger.destroyLogger(&node);
        }

        for (uint8_t i = 0; i < _pubCounter; i++)
        {
            if (_pubs[i] != NULL)
            {
                _pubs[i]->destroy(&node);
            }
        }

        for (uint8_t i = 0; i < _timerCounter; i++)
        {
            if (_timers[i] != NULL)
            {
                _timers[i]->destroy();
            }
        }

        for (uint8_t i = 0; i < _subCounter; i++)
        {
            if (_subs[i] != NULL)
            {
                _subs[i]->destroy(&node);
            }
        }

        REMOVE_WARN_UNUSED(rclc_executor_fini(&executor));
        REMOVE_WARN_UNUSED(rcl_node_fini(&node));
        REMOVE_WARN_UNUSED(rclc_support_fini(&support));
    }
}

#endif // __NODE_CPP__
