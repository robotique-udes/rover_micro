#ifndef J2_DEVICE_HPP
#define J2_DEVICE_HPP

#include "J2Actuator.hpp"

#include <rover_lib2/sensors/push_button.hpp>

#include "rover_can2/msgs/arm_joint_cmd.hpp"
#include "rover_can2/msgs/arm_joint_status.hpp"
#include "rover_can2/rover_can2.hpp"

DEFINE_LOG_NODE(J2Device, Logger::eNodeState::ON);

class J2Device
{
    static constexpr uint64_t LOOP_PERIOD_US = 1'000ULL;
    static constexpr float CAN_RECV_FREQ = 20.0F;
    static constexpr uint64_t CAN_WATCHDOG_VALIDITY_PERIOD = static_cast<uint64_t>(1'000.0F / CAN_RECV_FREQ * 2.0F);
    static constexpr float CAN_SEND_FREQ = 20.0F;
    static constexpr uint64_t CAN_SEND_PERIOD_MS = static_cast<uint64_t>(1'000.0F / CAN_SEND_FREQ);

    static constexpr float PUSH_BUTTON_SPEED_RAD_S = 0.10F;
    static constexpr float FULL_STOP_SPEED = 0.0F;
    static constexpr float CALIB_POSITION = 0.0F;

    static constexpr float FULL_STOP_SPEED_ERROR_TELORANCE = 0.01F;  // m/s

    using JointCanDeviceT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, J2Device>,
                                              RoverCan2::Publisher<RoverCan2::Msgs::ArmJointStatus>>;

  public:
    explicit J2Device(std::reference_wrapper<Stream> motorSerial_):
        _j2(motorSerial_)
    {
    }

    void init()
    {
        _j2.init();
        _j2.setSpeed(0.0F);
    }

    void update()
    {
        if (!_controlLoopTimer.isReady())
        {
            return;
        }

        if (_timerCanSend.isReady())
        {
            this->sendCanMsgs();
        }

        _j2.update();

        if (_pbCalib.isClicked())
        {
            _j2.setSpeed(FULL_STOP_SPEED);

            constexpr uint64_t CALIB_STOP_TIME = 1000ULL;
            OneShotTimer<uint64_t, &Time::millis> timerStop(CALIB_STOP_TIME);
            do
            {
                _j2.update();
                LOG_INFO(Logger::Nodes::J2Device, "_j2.getSpeed(): %f", _j2.getSpeed());

                if (!IN_ERROR(_j2.getSpeed(), FULL_STOP_SPEED_ERROR_TELORANCE, FULL_STOP_SPEED))
                {
                    timerStop = OneShotTimer<uint64_t, &Time::millis>(CALIB_STOP_TIME);
                }
            }
            while (!timerStop.isReady());

            _j2.calib(CALIB_POSITION);
        }

        if (_pbJogPlus.isClicked())
        {
            _j2.setSpeed(PUSH_BUTTON_SPEED_RAD_S);
        }
        else if (_pbJogNeg.isClicked())
        {
            _j2.setSpeed(-PUSH_BUTTON_SPEED_RAD_S);
        }
        else if (_j2CanWatchdog.isOk())
        {
            _j2.setSpeed(_j2SpeedGoal);
        }
        else
        {
            _j2.setSpeed(FULL_STOP_SPEED);
        }
    }

    JointCanDeviceT& getJointCanDevice()
    {
        return _j2CanDevice;
    }

  private:
    void sendCanMsgs()
    {
        RoverCan2::Msgs::ArmJointStatus j2Status;
        j2Status.data().currentPosition = _j2.getPosition();
        j2Status.data().currentSpeed = _j2.getSpeed();

        _j2CanDevice.sendMsg(j2Status);
    }

    void CB_J2Cmd(const RoverCan2::Msgs::ArmJointCmd& msgCan_)
    {
        _j2CanWatchdog.reset();
        _j2SpeedGoal = msgCan_.getData().targetSpeed;
    }

    PushButton _pbJogPlus = {PIN_PB_PLUS};
    PushButton _pbJogNeg = {PIN_PB_NEG};
    PushButton _pbCalib = {PIN_PB_CALIB};

    LoopTimer<uint64_t, &Time::micros> _controlLoopTimer = {LOOP_PERIOD_US};

    J2Actuator _j2;
    float _j2SpeedGoal = 0.0F;

    Watchdog<uint64_t, &Time::millis> _j2CanWatchdog = {CAN_WATCHDOG_VALIDITY_PERIOD};
    LoopTimer<uint64_t, &Time::millis> _timerCanSend = {CAN_SEND_PERIOD_MS};

    JointCanDeviceT _j2CanDevice
        = JointCanDeviceT(RoverCan2::Constant::eDeviceId::J2_CONTROLLER,
                          RoverCan2::SubscriberMember<RoverCan2::Msgs::ArmJointCmd, J2Device>(*this, &J2Device::CB_J2Cmd),
                          RoverCan2::Publisher<RoverCan2::Msgs::ArmJointStatus>());

    VALIDATE_CONCEPT(RoverObject, J2Device);
};

#endif
