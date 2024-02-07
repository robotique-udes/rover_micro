#ifndef __NODE_HPP__
#define __NODE_HPP__

#include "rover_ros_serial/rover_ros_serial.hpp"

namespace RoverRosSerial
{
    template <uint8_t nbSub>
    class Node
    {
    public:
        Node(HardwareSerial *serialPtr_ = &Serial, uint8_t heartbeatFrequency = 10u)
        {
            _pub_heartbeat = Heartbeat(heartbeatFrequency, serialPtr_);
        }

        ~Node() {}

        void spinSome()
        {
            _pub_heartbeat.update();
        }

    private:
        Heartbeat _pub_heartbeat;
        HardwareSerial *serialPtr_;
    };
}

#endif // __NODE_HPP__
