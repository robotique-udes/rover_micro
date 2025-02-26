#ifndef CAN_MANAGER_HPP
#define CAN_MANAGER_HPP

#include "rover_can2/can_device.hpp"
#include "rover_can2/can_driver.hpp"

#include "rover_lib2/rover_object.hpp"

#include <functional>

namespace RoverCan2
{
    template<size_t CAN_DEVICE_NB>
    class CanManager : public RoverObject
    {
      public:
        template<typename... Devices>
        CanManager(gpio_num_t ioRx, gpio_num_t ioTx, Devices&&... devices):
            _canDriver(ioRx, ioTx),
            _canDevices{std::ref(devices)...}
        {
            static_assert(sizeof...(Devices) == CAN_DEVICE_NB, "Number of devices must match template parameter");
        }

        void init() override
        {
            _canDriver.init();
        }

        void update() override
        {
            _canDriver.update();
            if (auto msg = _canDriver.getMsg())
            {
                for (std::reference_wrapper<CanDeviceBase> device : _canDevices)
                {
                    if (msg.value().canID == TO_UNDERLYING(device.get().getID()))
                    {
                        device.get().loadMessage(msg.value());
                    }
                }
            }
        }

      private:
        CanDriver _canDriver;
        std::array<std::reference_wrapper<CanDeviceBase>, CAN_DEVICE_NB> _canDevices;
    };

    template<typename... CanDevices>
    CanManager(gpio_num_t, gpio_num_t, CanDevices&&...) -> CanManager<sizeof...(CanDevices)>;

}  // namespace RoverCan2

#endif  // CAN_MANAGER_HPP
