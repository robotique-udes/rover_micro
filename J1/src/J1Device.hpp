#ifndef J34_DEVICE_HPP
#define J34_DEVICE_HPP

#include "J1Actuator.hpp"
#include "config.hpp"
#include "rover_can2/device.hpp"
#include "rover_lib2/sensors/push_button.hpp"

#include "rover_can2/msgs/arm_joint_cmd.hpp"
#include "rover_can2/msgs/arm_joint_status.hpp"
#include "rover_can2/rover_can2.hpp"

class J1Device
{
    static constexpr uint64_t LOOP_PERIOD_US = 250ULL;
    static constexpr float CAN_SEND_FREQ = 20.0F;
    static constexpr uint64_t CAN_SEND_PERIOD_MS = static_cast<uint64_t>(1'000.0F / CAN_SEND_FREQ);
    static constexpr float CAN_RECV_FREQ = 20.0F;
    static constexpr uint64_t CAN_WATCHDOG_VALIDITY_PERIOD = static_cast<uint64_t>(1'000.0F / CAN_RECV_FREQ * 2.0F);

    static constexpr float PUSH_BUTTON_SPEED_RAD_S = 0.10F;
    static constexpr float FULL_STOP_SPEED = 0.0F;
    static constexpr float CALIB_POSITION = 0.25F;

    static constexpr float FULL_STOP_SPEED_ERROR_TELORANCE = 0.01F;  // m

    using JointCanDeviceT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, J1Device>,
                                              RoverCan2::Publisher<RoverCan2::Msgs::ArmJointStatus>>;

  public:
    J1Device() = default;

    void init()
    {
        _j1.init();
        _j1.setSpeed(FULL_STOP_SPEED);
    }

    void update()
    {
        if (!_loopTimer.isReady())
        {
            return;
        }

        if (_timerCanSend.isReady())
        {
            sendCanMsgs();
        }

        _j1.update();

        if (_pbCalib.isClicked())
        {
            _j1.setSpeed(FULL_STOP_SPEED);

            constexpr uint64_t CALIB_STOP_TIME = 1000ULL;
            OneShotTimer<uint64_t, &Time::millis> timerStop(CALIB_STOP_TIME);
            do
            {
                _j1.update();

                if (!IN_ERROR(_j1.getSpeed(), FULL_STOP_SPEED_ERROR_TELORANCE, FULL_STOP_SPEED))
                {
                    timerStop = OneShotTimer<uint64_t, &Time::millis>(CALIB_STOP_TIME);
                }
            }
            while (!timerStop.isReady());

            _j1.calib(CALIB_POSITION);
        }

        if (_pbFwd.isClicked())
        {
            _j1.setSpeed(PUSH_BUTTON_SPEED_RAD_S);
        }
        else if (_pbRev.isClicked())
        {
            _j1.setSpeed(-PUSH_BUTTON_SPEED_RAD_S);
        }
        else
        {
            _j1.setSpeed(FULL_STOP_SPEED);
        }
    }

    JointCanDeviceT& getJ1Device()
    {
        return _j1CanDevice;
    }

  private:
    void sendCanMsgs()
    {
        float j1Pos = 0.0F;
        _j1.getPositions();

        float j1Speed = 0.0F;
        _j1.getSpeed();

        RoverCan2::Msgs::ArmJointStatus j1Status;
        j1Status.data().currentPosition = j1Pos;
        j1Status.data().currentSpeed = j1Speed;

        _j1CanDevice.sendMsg(j1Status);
    }

    void CB_J1Cmd(const RoverCan2::Msgs::ArmJointCmd& msg_)
    {
        _j1CanWatchdog.reset();
        _j1TargetSpeed = msg_.getData().targetSpeed;
    }

    LoopTimer<uint64_t, &Time::micros> _loopTimer = {LOOP_PERIOD_US};
    J1Actuator _j1;

    float _j1TargetSpeed = 0.0F;
    Watchdog<uint64_t, &Time::millis> _j1CanWatchdog = {CAN_WATCHDOG_VALIDITY_PERIOD};

    LoopTimer<uint64_t, &Time::millis> _timerCanSend = {CAN_SEND_PERIOD_MS};

    PushButton _pbCalib = {PIN_PB_CALIB};
    PushButton _pbFwd = {PIN_PB_FWD};
    PushButton _pbRev = {PIN_PB_REV};

    JointCanDeviceT _j1CanDevice
        = JointCanDeviceT(RoverCan2::Constant::eDeviceId::J1_CONTROLLER,
                          RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, J1Device>(*this, &J1Device::CB_J1Cmd),
                          RoverCan2::Publisher<RoverCan2::Msgs::ArmJointStatus>());

    VALIDATE_CONCEPT(RoverObject, J1Device);
};

#endif  // J34_DEVICE_HPP
