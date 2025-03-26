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
        void _init(void)
        {
            static_cast<Impl_T*>(this)->__init();
        }

        void _update(void)
        {
            static_cast<Impl_T*>(this)->__update();
        }

        bool sendMsg(const CanMsg& msg_)
        {
            return static_cast<Impl_T*>(this)->_sendMsg(msg_);
        }

        std::optional<CanMsg> getMsg(void)
        {
            return static_cast<Impl_T*>(this)->_getMsg();
        }

        size_t getAvailableMessagesNb(void) const
        {
            return static_cast<Impl_T*>(this)->_getAvailableMessagesNb();
        }
    };
}  // namespace RoverCan2

#endif  // CAN_DRIVER_HPP
