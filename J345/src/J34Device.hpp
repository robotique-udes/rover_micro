#ifndef J34_DEVICE_HPP
#define J34_DEVICE_HPP

#include "J34Actuator.hpp"
#include "config.hpp"
#include "rover_can2/device.hpp"
#include "rover_lib2/sensors/push_button.hpp"

#include "rover_can2/msgs/arm_joint_cmd.hpp"
#include "rover_can2/msgs/arm_joint_status.hpp"

#include "rover_can2/device.hpp"

class J34Device
{
    static constexpr uint64_t LOOP_PERIOD_US = 250ULL;
    static constexpr float CAN_SEND_FREQ = 20.0F;
    static constexpr uint64_t CAN_SEND_PERIOD_MS = static_cast<uint64_t>(1'000.0F / CAN_SEND_FREQ);
    static constexpr float CAN_RECV_FREQ = 20.0F;
    static constexpr uint64_t CAN_WATCHDOG_VALIDITY_PERIOD = static_cast<uint64_t>(1'000.0F / CAN_RECV_FREQ * 2.0F);

    using JointCanDeviceT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, J34Device>,
                                              RoverCan2::Publisher<RoverCan2::Msgs::ArmJointStatus>>;

  public:
    J34Device() = default;

    void init()
    {
        _j34.init();
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

        _j34.update();

        if (_pbCalib.isClicked())
        {
            _j34.calib(0.0F, 0.0F);
        }

        if (_pbFwd.isClicked())
        {
            _j34.setSpeeds(-10.0F, 0.0F);
        }
        else if (_pbRev.isClicked())
        {
            _j34.setSpeeds(10.0F, 0.0F);
        }
        else if (_pbRight.isClicked())
        {
            _j34.setSpeeds(0.0F, 10.0F);
        }
        else if (_pbLeft.isClicked())
        {
            _j34.setSpeeds(0.0F, -10.0F);
        }
        else if (_j3CanWatchdog.isOk() && _j4CanWatchdog.isOk())
        {
            _j34.setSpeeds(_j3TargetSpeed, _j4TargetSpeed);
        }
        else
        {
            _j34.setSpeeds(0.0F, 0.0F);
        }
    }

    JointCanDeviceT& getJ3Device()
    {
        return _j3CanDevice;
    }

    JointCanDeviceT& getJ4Device()
    {
        return _j4CanDevice;
    }

  private:
    void sendCanMsgs()
    {
        float j3Pos = 0.0F;
        float j4Pos = 0.0F;
        _j34.getPositions(j3Pos, j4Pos);

        float j3Speed = 0.0F;
        float j4Speed = 0.0F;
        _j34.getSpeeds(j3Speed, j4Speed);

        RoverCan2::Msgs::ArmJointStatus j3Status;
        j3Status.data().currentPosition = j3Pos;
        j3Status.data().currentSpeed = j3Speed;

        RoverCan2::Msgs::ArmJointStatus j4Status;
        j4Status.data().currentPosition = j4Pos;
        j4Status.data().currentSpeed = j4Speed;

        _j3CanDevice.sendMsg(j3Status);
        _j4CanDevice.sendMsg(j4Status);
    }

    void CB_J3Cmd(const RoverCan2::Msgs::ArmJointCmd& msg_)
    {
        _j3CanWatchdog.reset();
        _j3TargetSpeed = msg_.getData().targetSpeed;
    }

    void CB_J4Cmd(const RoverCan2::Msgs::ArmJointCmd& msg_)
    {
        _j4CanWatchdog.reset();
        _j4TargetSpeed = msg_.getData().targetSpeed;
    }

    LoopTimer<uint64_t, &Time::micros> _loopTimer = {LOOP_PERIOD_US};
    J34Actuator _j34;

    float _j3TargetSpeed = 0.0F;
    float _j4TargetSpeed = 0.0F;
    Watchdog<uint64_t, &Time::millis> _j3CanWatchdog = {CAN_WATCHDOG_VALIDITY_PERIOD};
    Watchdog<uint64_t, &Time::millis> _j4CanWatchdog = {CAN_WATCHDOG_VALIDITY_PERIOD};

    LoopTimer<uint64_t, &Time::millis> _timerCanSend = {CAN_SEND_PERIOD_MS};

    PushButton _pbCalib = {PIN_PB_J34_CALIB};
    PushButton _pbFwd = {PIN_PB_J3_FWD};
    PushButton _pbRev = {PIN_PB_J3_REV};
    PushButton _pbRight = {PIN_PB_J4_FWD};
    PushButton _pbLeft = {PIN_PB_J4_REV};

    JointCanDeviceT _j3CanDevice
        = JointCanDeviceT(RoverCan2::Constant::eDeviceId::GRIPPER_TILT_CONTROLLER,
                          RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, J34Device>(*this, &J34Device::CB_J3Cmd),
                          RoverCan2::Publisher<RoverCan2::Msgs::ArmJointStatus>());

    JointCanDeviceT _j4CanDevice
        = JointCanDeviceT(RoverCan2::Constant::eDeviceId::GRIPPER_ROT_CONTROLLER,
                          RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, J34Device>(*this, &J34Device::CB_J4Cmd),
                          RoverCan2::Publisher<RoverCan2::Msgs::ArmJointStatus>());

    VALIDATE_CONCEPT(RoverObject, J34Device);
};

#endif  // J34_DEVICE_HPP
