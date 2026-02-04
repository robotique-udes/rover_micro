#ifndef JR_DEVICE_HPP
#define JR_DEVICE_HPP

#include "JRActuator.hpp"
#include "config.hpp"
#include "rover_can2/device.hpp"
#include "rover_lib2/sensors/push_button.hpp"

#include "rover_can2/msgs/arm_joint_cmd.hpp"
#include "rover_can2/msgs/arm_joint_status.hpp"
#include "rover_can2/rover_can2.hpp"

class JRDevice
{
    static constexpr uint64_t LOOP_PERIOD_US = 1'000ULL;
    static constexpr float CAN_SEND_FREQ = 20.0F;
    static constexpr uint64_t CAN_SEND_PERIOD_MS = static_cast<uint64_t>(1'000.0F / CAN_SEND_FREQ);
    static constexpr float CAN_RECV_FREQ = 20.0F;
    static constexpr uint64_t CAN_WATCHDOG_VALIDITY_PERIOD = static_cast<uint64_t>(1'000.0F / CAN_RECV_FREQ * 2.0F);

    static constexpr float PUSH_BUTTON_SPEED_RAD_S = 0.10F;
    static constexpr float FULL_STOP_SPEED = 0.0F;
    static constexpr float CALIB_POSITION = 0.0F;

    static constexpr float FULL_STOP_SPEED_ERROR_TELORANCE = 0.01F;  // m

    using JointCanDeviceT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, JRDevice>,
                                              RoverCan2::Publisher<RoverCan2::Msgs::ArmJointStatus>>;

  public:
    JRDevice() = default;

    void init()
    {
        _jr.init();
        _jr.setSpeed(FULL_STOP_SPEED);
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

        _jr.update();

        if (_pbCalib.isClicked())
        {
            _jr.setSpeed(FULL_STOP_SPEED);

            constexpr uint64_t CALIB_STOP_TIME = 1000ULL;
            OneShotTimer<uint64_t, &Time::millis> timerStop(CALIB_STOP_TIME);
            do
            {
                _jr.update();

                if (!IN_ERROR(_jr.getSpeed(), FULL_STOP_SPEED_ERROR_TELORANCE, FULL_STOP_SPEED))
                {
                    timerStop = OneShotTimer<uint64_t, &Time::millis>(CALIB_STOP_TIME);
                }
            }
            while (!timerStop.isReady());

            _jr.calib(CALIB_POSITION);
        }

        if (_pbFwd.isClicked())
        {
            _jr.setSpeed(PUSH_BUTTON_SPEED_RAD_S);
        }
        else if (_pbRev.isClicked())
        {
            _jr.setSpeed(-PUSH_BUTTON_SPEED_RAD_S);
        }
        else if (_jrCanWatchdog.isOk())
        {
            _jr.setSpeed(_jrTargetSpeed);
        }
        else
        {
            _jr.setSpeed(FULL_STOP_SPEED);
        }
    }

    JointCanDeviceT& getJRDevice()
    {
        return _jrCanDevice;
    }

  private:
    void sendCanMsgs()
    {
        RoverCan2::Msgs::ArmJointStatus jrStatus;
        jrStatus.data().currentPosition = _jr.getPositions();
        jrStatus.data().currentSpeed = _jr.getSpeed();

        _jrCanDevice.sendMsg(jrStatus);
    }

    void CB_JRCmd(const RoverCan2::Msgs::ArmJointCmd& msg_)
    {
        _jrCanWatchdog.reset();
        _jrTargetSpeed = msg_.getData().targetSpeed;
    }

    LoopTimer<uint64_t, &Time::micros> _loopTimer = {LOOP_PERIOD_US};
    JRActuator _jr;

    float _jrTargetSpeed = 0.0F;
    Watchdog<uint64_t, &Time::millis> _jrCanWatchdog = {CAN_WATCHDOG_VALIDITY_PERIOD};

    LoopTimer<uint64_t, &Time::millis> _timerCanSend = {CAN_SEND_PERIOD_MS};

    PushButton _pbCalib = {PIN_PB_CALIB};
    PushButton _pbFwd = {PIN_PB_FWD};
    PushButton _pbRev = {PIN_PB_REV};

    JointCanDeviceT _jrCanDevice
        = JointCanDeviceT(RoverCan2::Constant::eDeviceId::JR_CONTROLLER,
                          RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, JRDevice>(*this, &JRDevice::CB_JRCmd),
                          RoverCan2::Publisher<RoverCan2::Msgs::ArmJointStatus>());

    VALIDATE_CONCEPT(RoverObject, JRDevice);
};

#endif  // JR_DEVICE_HPP
