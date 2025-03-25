#ifndef CAN_DRIVER_HPP
#define CAN_DRIVER_HPP

#include "rover_lib2/rover_object.hpp"
#include "rover_can2/can_msg.hpp"
#include <optional>

namespace RoverCan2
{
    template<typename Impl_T>
    class CanDriverBase : public RoverObject<CanDriverBase<Impl_T>>
    {
      private:
        friend Impl_T;
        CanDriverBase() = default;

      public:
        std::optional<CanMsg> sendMsg(const CanMsg& msg_)
        {
            return static_cast<Impl_T*>(this)->sendMsg(msg_);
        }

        std::optional<CanMsg> getMsg(void)
        {
            return static_cast<Impl_T*>(this)->getMsg();
        }

        size_t getAvailableMessagesNb(void) const
        {
            return static_cast<Impl_T*>(this)->getAvailableMessagesNb();
        }
    };
}  // namespace RoverCan2

#endif  // CAN_DRIVER_HPP
