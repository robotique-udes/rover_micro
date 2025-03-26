#ifndef CAN_DRIVER_MOCK_HPP
#define CAN_DRIVER_MOCK_HPP

#include "rover_can2/drivers/can_driver_base.hpp"
#include "rover_can2/msgs/test_msg.hpp"
#include "rover_lib2/helpers/circular_buffer.hpp"

using namespace RoverCan2;

class CanDriverMock : public CanDriverBase<CanDriverMock>
{
  public:
    void __init(void)
    {
        isInited = true;
    }

    void __update(void)
    {
        hasUpdated = true;
        // Must be done manually in tests
    }

    size_t _getAvailableMessagesNb(void) const
    {
        return newMsgsBuffer.size();
    }

    std::optional<CanMsg> _getMsg(void)
    {
        return newMsgsBuffer.getValue();
    }

    bool sendMsg(const CanMsg& msg_)
    {
        msgSentBuffer.addValue(msg_);
        return true;
    }

    CircularBuffer<CanMsg, 100> msgSentBuffer;
    CircularBuffer<CanMsg, 10> newMsgsBuffer;
    bool isInited = false;
    bool hasUpdated = false;
};

#endif  // CAN_DRIVER_MOCK_HPP
