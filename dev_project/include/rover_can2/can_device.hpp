#ifndef CAN_DEVICE_HPP
#define CAN_DEVICE_HPP

#include "rover_lib2/rover_object.hpp"
#include "rover_can2/subscriber.hpp"
#include "rover_can2/drivers/can_driver_base.hpp"

#include <array>
#include <optional>

namespace RoverCan2
{
    // Allows template shadowing for type validation
    class CanDeviceT
    {
      protected:
        CanDeviceT() = default;
    };

    template<typename... Subs>
    class CanDevice : public CanDeviceT
    {
        static constexpr size_t MAX_MSG_PARSED_PER_LOOP = 10U;

      public:
        explicit CanDevice(RoverCan2::Constant::eDeviceId id_, Subs&&... subs_):
            _id(id_),
            _subs{std::reference_wrapper<SubscriberBase>(subs_)...}
        {
        }

        bool parseMsg(const CanMsg& msgCan_)
        {
            bool success = true;
            if (msgCan_.getCanID() != this->getCanId())
            {
                // Not concerned but no error either
                return success;
            }

            for (auto& sub : _subs)
            {
                Msgs::Msg::eLoadMsgCode loadCode = sub.get().parseMsg(msgCan_);
                switch (loadCode)
                {
                    case Msgs::Msg::eLoadMsgCode::SUCCESS_COMPLETE:
                        break;
                    case Msgs::Msg::eLoadMsgCode::SUCCESS_INCOMPLETE:
                        break;
                    case Msgs::Msg::eLoadMsgCode::NOT_CONCERNED:
                        break;
                    case Msgs::Msg::eLoadMsgCode::ERROR_INVALID_MSG:
                        [[fallthrough]];
                    case Msgs::Msg::eLoadMsgCode::ERROR_MISSMATCH:
                        [[fallthrough]];
                    case Msgs::Msg::eLoadMsgCode::ERROR_IMPLEMENTATION:
                        success = false;
                        break;
                }
            }
            return success;
        }

        void sendMessage(const RoverCan2::Msgs::Msg& msg_) 
        {
            
        }

        constexpr RoverCan2::Constant::eDeviceId getCanId(void) const
        {
            return _id;
        }

      private:
        const RoverCan2::Constant::eDeviceId _id;
        const std::array<std::reference_wrapper<SubscriberBase>, sizeof...(Subs)> _subs;
    };

    template<typename... Subs>
    CanDevice(RoverCan2::Constant::eDeviceId id_, Subs&&...) -> CanDevice<Subs...>;

}  // namespace RoverCan2

#endif  // CAN_DEVICE_HPP
