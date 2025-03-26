#ifndef CAN_MANAGER_HPP
#define CAN_MANAGER_HPP

#include "rover_can2/can_device.hpp"
#include "rover_lib2/helpers/log.hpp"
#include "rover_lib2/helpers/watchdog.hpp"

#include <array>
#include <tuple>

DEFINE_LOG_NODE(CanManager, Logger::eNodeState::OFF);

namespace RoverCan2
{
    template<typename CanDriverT, typename... DevicesT>
    class CanManager : public RoverObject<CanManager<CanDriverT, DevicesT...>>
    {
        static_assert(std::is_base_of_v<CanDriverBase<CanDriverT>, CanDriverT>, "CanDriverT must be of type CanDriverBase");
        static_assert((std::is_base_of_v<CanDeviceT, std::remove_reference_t<DevicesT>> && ...),
                      "All DevicesT... must be of type CanDevice");

        static constexpr uint8_t MAX_MSG_PARSE_PER_UPDATE = 5U;
        static constexpr unsigned long MASTER_HEARTBEAT_FREQUENCY_HZ = 10UL;
        static constexpr unsigned long MASTER_WATCHDOG_PERIOD_MS = 1'000UL * 1UL / (MASTER_HEARTBEAT_FREQUENCY_HZ / 2UL);

      public:
        CanManager(CanDriverT& driver_, DevicesT&&... devices_):
            _driver(driver_),
            _canDevices(std::forward<DevicesT>(devices_)...),
            watchdog(MASTER_WATCHDOG_PERIOD_MS)
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

                if (msgOpt.value().getCanID() == Constant::eDeviceId::MASTER_COMPUTER_UNIT)
                {
                }

                this->parseMsgAllDevices(msgOpt.value());
            }
        }

        bool sendMsg(Constant::eDeviceId senderId_, const Msgs::Msg& msg_, bool sendEvenIfDeviceIdInvalid_ = false)
        {
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

        void processMsgFromMaster(const CanMsg& msg_)
        {
            switch (msg_._msgID)
            {
                case Constant::eMsgId::ERROR_STATE:
#warning TODO
                    break;
                case Constant::eMsgId::HEARTBEAT:
#warning TODO
                    break;
            }
        }

        CanDriverT& _driver;
        std::tuple<DevicesT...> _canDevices;
        Watchdog<unsigned long, millis> watchdog;
    };

    template<typename CanDriverT, typename... DevicesT>
    CanManager(CanDriverT canDriver_, DevicesT&&...) -> CanManager<CanDriverT, DevicesT...>;

}  // namespace RoverCan2

#endif  // CAN_MANAGER_HPP
