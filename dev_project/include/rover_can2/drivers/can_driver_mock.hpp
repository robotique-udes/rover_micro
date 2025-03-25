#ifndef CAN_DRIVER_MOCK_HPP
#define CAN_DRIVER_MOCK_HPP

#include "rover_can2/drivers/can_driver_base.hpp"
#include "rover_can2/msgs/test_msg.hpp"
#include "rover_lib2/helpers/circular_buffer.hpp"

using namespace RoverCan2;

class CanDriverMock : public CanDriverBase<CanDriverMock>
{
  public:
    void init(void)
    {
        isInited = true;
    }

    void update(void)
    {
        hasUpdated = true;
        // Must be done manually in tests
    }

    size_t getAvailableMessagesNb(void) const
    {
        return _newMsgsBuffer.size();
    }

    std::optional<CanMsg> getMsg(void)
    {
        return _newMsgsBuffer.getValue();
    }

    CircularBuffer<CanMsg, 10> _newMsgsBuffer;
    bool isInited = false;
    bool hasUpdated = false;
};

#endif  // CAN_DRIVER_MOCK_HPP
