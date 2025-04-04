#ifndef CAN_DEVICE_HPP
#define CAN_DEVICE_HPP

#include "rover_can2/constant.hpp"
#include "rover_can2/msgs/msg.hpp"
#include "rover_can2/publisher.hpp"
#include "rover_can2/subscriber.hpp"

#include <tuple>
#include <type_traits>

namespace RoverCan2
{
    // Allows template shadowing for type validation
    class CanDeviceT
    {
      protected:
        CanDeviceT() = default;
    };

    template<typename... Pubs_SubsT>
    class CanDevice : public CanDeviceT
    {
        // clang-format off
        static_assert((... && (std::is_base_of_v<SubscriberBaseT, std::remove_reference_t<Pubs_SubsT>> 
                              || std::is_base_of_v<PublisherBaseT, std::remove_reference_t<Pubs_SubsT>>)),
                      "All template arguments must be derived from SubscriberBaseT or PublisherBaseT");
        // clang-format on

        static constexpr size_t MAX_MSG_PARSED_PER_LOOP = 10U;

      public:
        explicit CanDevice(RoverCan2::Constant::eDeviceId id_, Pubs_SubsT&&... subs_):
            _id(id_),
            _pubs_subs(std::forward<Pubs_SubsT>(subs_)...)
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
                [&](Pubs_SubsT&... sub_)
                {
                    ((success &= this->loadMsgSub(sub_, msgCan_)), ...);
                },
                _pubs_subs);

            return success;
        }

        template<typename ManagerT>
        bool sendPubQueuedMsgs(ManagerT& manager_)
        {
            VALIDATE_BASE_TYPE(CanManagerT, ManagerT);

            bool allSuccess = true;
            std::apply(
                [&](Pubs_SubsT&... pub_)
                {
                    ((allSuccess &= this->sendPubMsg(manager_, pub_)), ...);
                },
                _pubs_subs);

            return allSuccess;
        }

        constexpr RoverCan2::Constant::eDeviceId getCanId(void) const
        {
            return _id;
        }

      private:
        template<typename SubT>
        bool loadMsgSub(SubT& sub_, const CanMsg& msgCan_)
        {
            if constexpr (std::is_base_of_v<SubscriberBaseT, std::remove_reference_t<decltype(sub_)>>)
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

            return true;  // Not concerned
        }

        template<typename ManagerT, typename PubT>
        bool sendPubMsg(ManagerT& manager_, PubT& pub_)
        {
            VALIDATE_BASE_TYPE(CanManagerT, ManagerT);

            if constexpr (std::is_base_of_v<PublisherBaseT, std::remove_reference_t<decltype(pub_)>>)
            {
                return pub_.sendQueuedMsgs(this->getCanId(), manager_);
            }

            return true;  // Not concerned
        }

        const RoverCan2::Constant::eDeviceId _id;
        std::tuple<Pubs_SubsT...> _pubs_subs;
    };

    template<typename... Pubs_SubsT>
    CanDevice(RoverCan2::Constant::eDeviceId id_, Pubs_SubsT&&...) -> CanDevice<Pubs_SubsT...>;

}  // namespace RoverCan2

#endif  // CAN_DEVICE_HPP
