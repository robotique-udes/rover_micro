#ifndef __NODE_HPP__
#define __NODE_HPP__

#if !defined (ESP32)
#error CPU not supported
#endif

#include "micro_ros_platformio.h"
#include <rcl/rcl.h>
#include <rclc/executor.h>

#include "rover_micro_ros_lib/rover_micro_ros_lib.hpp"
#include "helpers/helpers.hpp"

namespace RoverMicroRosLib
{
    template <uint8_t NB_PUBLISHER, uint8_t NB_TIMER, uint8_t NB_SUBSCRIBER>
    class Node
    {
    public:
        enum eConnectionStates : uint8_t
        {
            WAITING_AGENT,
            AGENT_AVAILABLE,
            AGENT_CONNECTED,
            AGENT_DISCONNECTED
        };

        Node(const char *namespaceName_,
             const char *nodeName_,
             bool withLogger_ = true,
             bool withLED_ = true,
             uint8_t ledPIN_ = LED_BUILTIN,
             uint32_t connectionValidationInterval_ = 200UL,
             uint32_t reconnectionInterval_ = 500UL);
        ~Node(void);
        void init(void);
        void addPublisher(Publisher *pub_);
        void addTimer(Timer *timer_);
        void addSubscriber(Subscriber *sub_);
        void spinSome(const uint32_t timeoutNs_ = 0UL);
        uint8_t getConnectionState(void);
        void updateLED(void);

    private:
        eConnectionStates _connectionState = eConnectionStates::WAITING_AGENT;
        TimerMillis _timerCheckReconnect;
        TimerMillis _timerCheckDisconnect;
        bool _withLED;
        uint8_t _ledPIN;

        rclc_support_t support;
        rcl_node_t node;
        rclc_executor_t executor;
        rcl_allocator_t allocator;
        const char *_nodeName = NULL;
        const char *_namespaceName = NULL;
        Publisher *_pubs[NB_PUBLISHER];
        uint8_t _pubCounter = 0;
        Timer *_timers[NB_TIMER];
        uint8_t _timerCounter = 0;
        Subscriber *_subs[NB_SUBSCRIBER];
        uint8_t _subCounter = 0;
        bool _withLogger = true;

        bool createEntities(void);
        void destroyEntities(void);
    };
}
#include "rover_micro_ros_lib/node.cpp"

#endif // __NODE_HPP__
