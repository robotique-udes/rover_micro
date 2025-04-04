#ifndef CAN_DEVICE_HPP
#define CAN_DEVICE_HPP

#include "rover_can2/constant.hpp"
#include "rover_can2/msgs/msg.hpp"

#include <tuple>
#include <type_traits>

#warning TODO: Check all template args for type validation -> Maybe macro for assert?

namespace RoverCan2
{
    // Allows template shadowing for type validation
    class CanDeviceT
    {
      protected:
        CanDeviceT() = default;
    };

    template<typename... SubsT>
    class CanDevice : public CanDeviceT
    {
        static constexpr size_t MAX_MSG_PARSED_PER_LOOP = 10U;

      public:
        explicit CanDevice(RoverCan2::Constant::eDeviceId id_, SubsT&&... subs_):
            _id(id_),
            _subs(std::forward<SubsT>(subs_)...)
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

            std::apply(
                [&](SubsT&... sub)
                {
                    ((success &= this->loadMsgSub(sub, msgCan_)), ...);
                },
                _subs);

            return success;
        }

        constexpr RoverCan2::Constant::eDeviceId getCanId(void) const
        {
            return _id;
        }

      private:
        template<typename SubT>
        bool loadMsgSub(SubT& sub_, const CanMsg& msgCan_)
        {
            bool success = true;
            Msgs::eLoadMsgCode loadCode = sub_.parseMsg(msgCan_);
            switch (loadCode)
            {
                case Msgs::eLoadMsgCode::SUCCESS_COMPLETE:
                    break;
                case Msgs::eLoadMsgCode::SUCCESS_INCOMPLETE:
                    break;
                case Msgs::eLoadMsgCode::NOT_CONCERNED:
                    break;
                case Msgs::eLoadMsgCode::ERROR_INVALID_MSG:
                    [[fallthrough]];
                case Msgs::eLoadMsgCode::ERROR_MISSMATCH:
                    [[fallthrough]];
                case Msgs::eLoadMsgCode::ERROR_IMPLEMENTATION:
                    success = false;
                    break;
            }

            return success;
        }

        const RoverCan2::Constant::eDeviceId _id;
        std::tuple<SubsT...> _subs;
    };

    template<typename... SubsT>
    CanDevice(RoverCan2::Constant::eDeviceId id_, SubsT&&...) -> CanDevice<SubsT...>;

}  // namespace RoverCan2

#endif  // CAN_DEVICE_HPP
