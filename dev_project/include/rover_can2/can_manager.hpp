#ifndef CAN_MANAGER_HPP
#define CAN_MANAGER_HPP

#include "rover_can2/msgs/error_state.hpp"
#include "rover_can2/subscriber.hpp"

#include "rover_can2/drivers/can_driver_base.hpp"
#include "rover_can2/can_device.hpp"
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/rover_object.hpp"
#include "rover_lib2/helpers/health_state.hpp"

#include <tuple>
#include <optional>

DEFINE_LOG_NODE(CanManager, Logger::eNodeState::OFF);

namespace RoverCan2
{
    // Allows shadow type validation
    class CanManagerT
    {
      protected:
        CanManagerT() = default;
    };
    template<typename CanDriverT, typename... DevicesT>
    class CanManager : public RoverObject<CanManager<CanDriverT, DevicesT...>>,
                       CanManagerT
    {
        VALIDATE_BASE_TYPE(Drivers::CanDriverBaseT, CanDriverT);
        VALIDATE_BASE_TYPE_PACK(CanDeviceT, DevicesT);

        static_assert((std::is_base_of_v<CanDeviceT, std::remove_reference_t<DevicesT>> && ...),
                      "All DevicesT... must be of type CanDevice");

        static constexpr uint8_t MAX_MSG_PARSE_PER_UPDATE = 10U;
        static constexpr unsigned long MASTER_HEARTBEAT_FREQUENCY_HZ = 10UL;
        static constexpr Constant::eDeviceId ERROR_STATE_HANDLER_ID = Constant::eDeviceId::MASTER_COMPUTER_UNIT;

      public:
        CanManager(CanDriverT& driver_, DevicesT&&... devices_):
            _canDevices(std::forward<DevicesT>(devices_)...),
            _driver(driver_),
            _dev_Master(ERROR_STATE_HANDLER_ID,
                        SubscriberMember<Msgs::ErrorState, CanManager<CanDriverT, DevicesT...>>{
                            *this,
                            &CanManager<CanDriverT, DevicesT...>::CB_ErrorStateFromMaster})
        {
        }

        void _init(void)
        {
            _driver.init();
        }

        void _update(void)
        {
            _driver.update();

            std::optional<CanMsg> msgOpt = std::nullopt;
            for (uint8_t i = 0U; i < MAX_MSG_PARSE_PER_UPDATE; i++)
            {
                msgOpt = _driver.getMsg();
                if (!msgOpt.has_value())
                {
                    break;
                }

                this->parseMsgAllDevices(msgOpt.value());
            }

            this->publishAllQueuedMsgs();
        }

        template<typename MsgT>
        bool sendMsg(Constant::eDeviceId senderId_, const MsgT& msg_, bool sendEvenIfDeviceIdInvalid_ = false)
        {
            static_assert(std::is_base_of_v<Msgs::Msg<MsgT>, MsgT>,
                          "MsgT template argument must be CRTP child type of Msgs::Msg");

            if (!sendEvenIfDeviceIdInvalid_)
            {
                bool senderIdValid = false;
                std::apply(
                    [&](DevicesT&&... devices_)
                    {
                        ((senderIdValid |= (senderId_ == devices_.getCanId())), ...);
                    },
                    _canDevices);

                if (!senderIdValid)
                {
                    LOG_WARN(
                        Logger::Nodes::CanManager,
                        "Trying to send a can msg from device ID: %u, but this manager doesn't contain a device with this ID",
                        TO_UNDERLYING(senderId_));
                    return false;
                }
            }

            bool success = false;
            uint8_t nbOfMsgToSend = msg_.getMsgContentCount();
            for (uint8_t i = 0U; i < nbOfMsgToSend; i++)
            {
                std::optional<CanMsg> canMsgOpt = msg_.getCanMsg(i);
                if (!canMsgOpt)
                {
                    success = false;
                    continue;
                }

                canMsgOpt.value().setCanID(senderId_);
                RoverCan2::CanMsg canMsg = canMsgOpt.value();
                success = std::min(success, _driver.sendMsg(canMsg));
            }

            return success;
        }

      protected:
        std::tuple<DevicesT...> _canDevices;

      private:
        bool parseMsgAllDevices(const CanMsg& msg_)
        {
            bool allSuccess = true;

            std::apply(
                [&](DevicesT&&... devices_)
                {
                    ((allSuccess &= parseMsgSingleDevices(devices_, msg_)), ...);
                },
                _canDevices);

            allSuccess &= parseMsgSingleDevices(_dev_Master, msg_);

            return allSuccess;
        }

        template<typename deviceT>
        bool parseMsgSingleDevices(deviceT& device_, const CanMsg& msg_)
        {
            bool success = false;
            success = device_.parseMsg(msg_);
            if (!success)
            {
                LOG_WARN(Logger::Nodes::CanManager,
                         "Error from CanDevice of ID: %u, while load message of type: %u",
                         device_.getCanId(),
                         TO_UNDERLYING(msg_.getMsgID()));
            }
            return success;
        }

        void CB_ErrorStateFromMaster(const Msgs::ErrorState& /*msg_*/)
        {
            std::apply(
                [&](DevicesT&... device)
                {
                    ((this->sendDeviceErrorState(device)), ...);
                },
                _canDevices);
        }

        template<typename DeviceT>
        void sendDeviceErrorState(DeviceT& device_)
        {
            static_assert(std::is_base_of_v<CanDeviceT, DeviceT>, "Template parameter must be of type CanDeviceT");

            Msgs::ErrorState msg;
            msg.data().error = HealthState::getInstance().getInError();

            this->sendMsg(device_.getCanId(), msg);
        }

        void publishAllQueuedMsgs(void)
        {
            std::apply(
                [&](DevicesT&... devices_)
                {
                    ((devices_.sendPubsQueuedMsgs(*this)), ...);
                },
                _canDevices);
        }

        CanDriverT& _driver;
        CanDevice<SubscriberMember<Msgs::ErrorState, CanManager<CanDriverT, DevicesT...>>> _dev_Master;
    };

    template<typename CanDriverT, typename... DevicesT>
    CanManager(CanDriverT canDriver_, DevicesT&&...) -> CanManager<CanDriverT, DevicesT...>;

}  // namespace RoverCan2

#endif  // CAN_MANAGER_HPP
