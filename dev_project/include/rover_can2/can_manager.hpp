#ifndef CAN_MANAGER_HPP
#define CAN_MANAGER_HPP

#include "rover_can2/can_device.hpp"
#include "rover_lib2/helpers/log.hpp"

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

      public:
        CanManager(CanDriverT& driver_, DevicesT&&... devices_):
            _driver(driver_),
            _canDevices(std::forward<DevicesT>(devices_)...)
        {
        }

        void init(void)
        {
            _driver.init();
        }

        void update(void)
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
        }

      private:
        CanDriverT& _driver;
        std::tuple<DevicesT...> _canDevices;

        bool parseMsgAllDevices(const CanMsg& msg_)
        {
            bool allSuccess = true;

            std::apply(
                [&](auto&... devices_)
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
                         TO_UNDERLYING(msg_.msgID));
            }
            return success;
        }
    };

    template<typename CanDriverT, typename... DevicesT>
    CanManager(CanDriverT canDriver_, DevicesT&&...) -> CanManager<CanDriverT, DevicesT...>;

}  // namespace RoverCan2

#endif  // CAN_MANAGER_HPP
