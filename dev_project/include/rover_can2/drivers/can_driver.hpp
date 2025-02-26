#ifndef CAN_DRIVER_HPP
#define CAN_DRIVER_HPP

#include "rover_lib2/rover_object.hpp"
#include "rover_can2/can_msg.hpp"
#include <optional>

namespace RoverCan2
{
    class CanDriver : public RoverObject
    {
      public:
        virtual std::optional<CanMsg> sendMsg(const CanMsg&) = 0;
        virtual std::optional<CanMsg> getMsg(void) = 0;
        virtual size_t getAvailableMessagesNb(void) = 0;
    };

    // class CanDriverMock : public CanDriverBase
    // {
    //   public:
    //     virtual std::optional<CanMsg> sendMsg(const CanMsg&) = 0;
    //     virtual std::optional<CanMsg> getMsg(void) = 0;
    //     virtual size_t getAvailableMessagesNb(void) = 0;
    // };

}  // namespace RoverCan2

#endif  // CAN_DRIVER_HPP
