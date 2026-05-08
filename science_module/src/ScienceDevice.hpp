#ifndef SCIENCE_DEVICE_HPP
#define SCIENCE_DEVICE_HPP

#include "LinActuator.hpp"
#include "config.hpp"
#include "rover_can2/device.hpp"
#include "rover_lib2/sensors/push_button.hpp"

#include "rover_can2/msgs/science.hpp"
#include "rover_can2/rover_can2.hpp"

class ScienceDevice
{
    static constexpr uint64_t LOOP_PERIOD_US = 250ULL;
    static constexpr float CAN_SEND_FREQ = 20.0F;
    static constexpr uint64_t CAN_SEND_PERIOD_MS = static_cast<uint64_t>(1'000.0F / CAN_SEND_FREQ);
    static constexpr float CAN_RECV_FREQ = 20.0F;
    static constexpr uint64_t CAN_WATCHDOG_VALIDITY_PERIOD = static_cast<uint64_t>(1'000.0F / CAN_RECV_FREQ * 2.0F);

    static constexpr float PUSH_BUTTON_SPEED_RAD_S = 0.5F;
    static constexpr float FULL_STOP_SPEED = 0.0F;
    static constexpr float CALIB_POSITION = 0.0F;

    static constexpr float FULL_STOP_SPEED_ERROR_TELORANCE = 0.01F;  // m

    using DeviceT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::Science, ScienceDevice>>;

  public:
    ScienceDevice() = default;

    void init()
    {
        _linAct.init();
        _linAct.setSpeed(FULL_STOP_SPEED);
    }

    void update()
    {
        if (!_loopTimer.isReady())
        {
            return;
        }

        _linAct.update();
    }

    DeviceT& getScienceDevice()
    {
        return _scienceCanDevice;
    }

  private:
    void CB_ScienceCmd(const RoverCan2::Msgs::Science& msg_)
    {
        _scienceCanWatchdog.reset();
        // _j1TargetSpeed = msg_.getData().targetSpeed;
    }

    LoopTimer<uint64_t, &Time::micros> _loopTimer = {LOOP_PERIOD_US};
    LinearAct _linAct;

    float _linActTargetSpeed = 0.0F;
    Watchdog<uint64_t, &Time::millis> _scienceCanWatchdog = {CAN_WATCHDOG_VALIDITY_PERIOD};

    LoopTimer<uint64_t, &Time::millis> _timerCanSend = {CAN_SEND_PERIOD_MS};

    DeviceT _scienceCanDevice
        = DeviceT(RoverCan2::Constant::eDeviceId::SCIENCE,
                  RoverCan2::SubscriberMember<RoverCan2::Msgs::Science, ScienceDevice>(*this, &ScienceDevice::CB_ScienceCmd));
    VALIDATE_CONCEPT(RoverObject, ScienceDevice);
};

#endif  // J34_DEVICE_HPP
