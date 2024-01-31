#ifndef __ROVER_ROS_SERIAL_HPP__
#define __ROVER_ROS_SERIAL_HPP__

#include <stdint.h>
#include "helpers/log.h"

#define ROVER_ROS_SERIAL

class rover_ros_serial
{
public:
    enum eHeaderCode : uint8_t
    {
        BEGIN = 128,
        publisher = 130,
        subscriber = 150,
        config = 170,
        eLast
    };
};

class Msg
{
public:
    virtual uint8_t *getSerializedData(void) = 0;
    virtual uint8_t getSerializedDataSize(void) = 0;
};

class rover_ros_serial__msg__Logger : protected Msg
{
public:
    struct s__rover_ros_serial__msg__Logger
    {
        uint8_t begin;
        uint8_t length;
        char msg[98];
    };

    union u__rover_ros_serial__msg__Logger
    {
        s__rover_ros_serial__msg__Logger msg;
        uint8_t data[sizeof(s__rover_ros_serial__msg__Logger)];
    };

    uint8_t _sizeData = sizeof(u__rover_ros_serial__msg__Logger::data);

public:
    rover_ros_serial__msg__Logger()
    {
        FOR_ALL(msg.msg)
        {
            msg.msg[i] = '\0';
        }
    }
    ~rover_ros_serial__msg__Logger() {}

    uint8_t *getSerializedData(void)
    {
        uData.msg = msg;
        return uData.data;
    }

    uint8_t getSerializedDataSize(void)
    {
        return _sizeData;
    }

    s__rover_ros_serial__msg__Logger msg;
    u__rover_ros_serial__msg__Logger uData;
};

#endif // __ROVER_ROS_SERIAL_HPP__
