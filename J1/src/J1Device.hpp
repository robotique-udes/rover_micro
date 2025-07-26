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

    static constexpr float PUSH_BUTTON_SPEED_RAD_S = 0.1F;

    using JointCanDeviceT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, J1Device>,
                                              RoverCan2::Publisher<RoverCan2::Msgs::ArmJointStatus>>;

  public:
    J1Device() = default;

    void init()
    {
        _j1.init();
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
            _j1.calib(0.0F);
        }

        if (_pbFwd.isClicked())
        {
            _j1.setSpeed(PUSH_BUTTON_SPEED_RAD_S);
        }
        else if (_pbRev.isClicked())
        {
            LOG_DEBUG(Logger::Nodes::J1Actuator, "Push button reverse clicked");
            _j1.setSpeed(-PUSH_BUTTON_SPEED_RAD_S);
        }
        else
        {
            _j1.setSpeed(0.0F);
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
        _j1.getPositions(j1Pos);

        float j1Speed = 0.0F;
        _j1.getSpeed(j1Speed);

        RoverCan2::Msgs::ArmJointStatus j1Status;
        j1Status.data().currentPosition = j1Pos;
        j1Status.data().currentSpeed = j1Speed;

        _j1CanDevice.sendMsg(j1Status);
    }

    void CB_J3Cmd(const RoverCan2::Msgs::ArmJointCmd& msg_)
    {
        _j1CanWatchdog.reset();
        _j1TargetSpeed = msg_.getData().targetSpeed;
    }

    void CB_J4Cmd(const RoverCan2::Msgs::ArmJointCmd& msg_)
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
        = JointCanDeviceT(RoverCan2::Constant::eDeviceId::GRIPPER_TILT_CONTROLLER,
                          RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, J1Device>(*this, &J1Device::CB_J3Cmd),
                          RoverCan2::Publisher<RoverCan2::Msgs::ArmJointStatus>());

    VALIDATE_CONCEPT(RoverObject, J1Device);
};

#endif  // J34_DEVICE_HPP
