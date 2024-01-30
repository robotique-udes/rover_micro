#ifndef __ROVER_MSGS__MSG__ANTENNA_CMD_HPP__
#define __ROVER_MSGS__MSG__ANTENNA_CMD_HPP__

#include "rover_ros_serial.hpp"

class rover_msgs__msg__AntennaCmd : protected Msg
{
private:
    struct s__rover_msgs__msg__AntennaCmd
    {
        uint8_t header;
        bool status;
        float speed;
        uint8_t ofl;
    };

    union u__rover_msgs__msg__AntennaCmd
    {
        s__rover_msgs__msg__AntennaCmd msg;
        uint8_t data[sizeof(s__rover_msgs__msg__AntennaCmd)];
    };

    uint8_t _sizeData = sizeof(u__rover_msgs__msg__AntennaCmd::data);

public:
    rover_msgs__msg__AntennaCmd() {}
    ~rover_msgs__msg__AntennaCmd() {}

    uint8_t *getSerializedData(void)
    {
        uData.msg = msg;
        return uData.data;
    }

    uint8_t getSerializedDataSize(void)
    {
        return _sizeData;
    }

    s__rover_msgs__msg__AntennaCmd msg;
    u__rover_msgs__msg__AntennaCmd uData;
};

#endif // __ROVER_MSGS__MSG__ANTENNA_CMD_HPP__
