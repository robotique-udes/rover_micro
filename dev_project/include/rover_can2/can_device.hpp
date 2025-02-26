#ifndef CAN_DEVICE_HPP
#define CAN_DEVICE_HPP

#include "rover_can2/subscriber.hpp"
#include "rover_can2/drivers/can_driver.hpp"

#include <array>
#include <optional>

namespace RoverCan2
{
    template<size_t SUB_NB>
    class CanDevice : public RoverObject
    {
      public:
        CanDevice(RoverCan2::Constant::eDeviceId id_, std::array<std::reference_wrapper<SubscriberBase>, SUB_NB> subs_):
            _id(id_),
            _subs(subs_),
            _canDriver(std::nullopt)
        {
        }

        virtual void init(void) override {}
        virtual void update(void) override {}

        bool parseMsg(const CanMsg& msgCan_)
        {
            bool hadErrors = false;
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
                    case Msgs::Msg::eLoadMsgCode::ERROR_MISSMATCH:
                        [[fallthrough]];
                    case Msgs::Msg::eLoadMsgCode::ERROR_IMPLEMENTATION:
                        hadErrors = true;
                        break;
                }
            }

            return hadErrors;
        }

        /**
         * @brief Attach a can driver to the device for message sending
         *
         * @attention [WARNING] Has to be called before trying to send message
         * otherwise they won't be sent
         * @param driver_
         */
        void attachDriver(const RoverCan2::CanDriver& driver_)
        {
            _canDriver = driver_;
        }

        void sendMessage(const RoverCan2::Msgs::Msg& msg_)
        {
            msg_.
        }

      private:
        constexpr RoverCan2::Constant::eDeviceId getCanId(void)
        {
            return _id;
        }

        const RoverCan2::Constant::eDeviceId _id;
        const std::array<std::reference_wrapper<SubscriberBase>, SUB_NB> _subs;
        std::optional<std::reference_wrapper<RoverCan2::CanDriver>> _canDriver;
    };
}  // namespace RoverCan2

#endif  // CAN_DEVICE_HPP
