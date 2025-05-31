#ifndef YAW_CONTROLLER_HPP
#define YAW_CONTROLLER_HPP

#include "device_config.hpp"

#include <rover_can2/rover_can2.hpp>
#include <rover_can2/msgs/PTZ_cmd.hpp>
#include <rover_can2/msgs/PTZ_status.hpp>
#include <rover_can2/msgs/PTZ_config.hpp>

#include <rover_lib2/actuators/servo.hpp>
#include <rover_lib2/actuators/PWM_generators/MCPWM.hpp>
#include <rover_lib2/helpers/loop_timer.hpp>

class GimbalController : public RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::PtzCmd, GimbalController>,
                                                  RoverCan2::Publisher<RoverCan2::Msgs::PtzStatus>,
                                                  RoverCan2::SubscriberMember<RoverCan2::Msgs::PtzConfig, GimbalController>>,
                         public RoverObject<GimbalController>
{
    using DeviceT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::PtzCmd, GimbalController>,
                                      RoverCan2::Publisher<RoverCan2::Msgs::PtzStatus>,
                                      RoverCan2::SubscriberMember<RoverCan2::Msgs::PtzConfig, GimbalController>>;

    static constexpr float PTZ_STATUS_SEND_FREQUENCY = 5.0F;
    static constexpr uint64_t PTZ_STATUS_SEND_PERIOD_MS = static_cast<uint64_t>(1'000.0F / PTZ_STATUS_SEND_FREQUENCY);

  public:
    GimbalController():
        DeviceT(DEVICE_ID,
                RoverCan2::SubscriberMember<RoverCan2::Msgs::PtzCmd, GimbalController>(*this, &GimbalController::CB_CAN_PTZCmd),
                RoverCan2::Publisher<RoverCan2::Msgs::PtzStatus>(),
                RoverCan2::SubscriberMember<RoverCan2::Msgs::PtzConfig, GimbalController>(*this,
                                                                                          &GimbalController::CB_CAN_PTZConfig)),
        _timerPTZStatusSend(PTZ_STATUS_SEND_PERIOD_MS)
    {
    }

    void _init(void)
    {
        _panServo.init();
    }

    void _update(void)
    {
        _panServo.update();

        if (_timerPTZStatusSend.isReady())
        {
            RoverCan2::Msgs::PtzStatus status;
            status.data().pan = _panServo.getPosition();
            status.data().tilt = 0.0F;  // Not supported
            status.data().zoom = 1.0F;  // Not supported

            this->sendMsg(status);
        }
    }

  private:
    void CB_CAN_PTZCmd(const RoverCan2::Msgs::PtzCmd& ctrlMsg_)
    {
        _panServo.setPosition(ctrlMsg_.getData().pan);
    }

    void CB_CAN_PTZConfig(const RoverCan2::Msgs::PtzConfig& configMsg_)
    {
        std::optional<float> minPanLimit = std::nullopt;
        if (configMsg_.getData().panMinPosition != 0.0F)
        {
            minPanLimit = float();
            minPanLimit = configMsg_.getData().panMinPosition;
        }

        std::optional<float> maxPanLimit = std::nullopt;
        if (configMsg_.getData().panMaxPosition != 0.0F)
        {
            maxPanLimit = float();
            maxPanLimit = configMsg_.getData().panMaxPosition;
        }

        _panServo.setJointLimit(minPanLimit, maxPanLimit);
        _panServo.setMaxSpeed(configMsg_.getData().panMaxSpeed);
    }

    PWMGenerators::MCPWMTimer __pwmGenTimer = PWMGenerators::MCPWMTimer(GET_SERVO_TIMING_CONFIG<eServoType::PAN>().frequency,
                                                                        PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0);
    PWMGenerators::MCPWM __panServoPwmGen = PWMGenerators::MCPWM(PIN_SERVO_1,
                                                                 __pwmGenTimer,
                                                                 PWMGenerators::MCPWM::ePinOutputMode::ACTIVE_HIGH,
                                                                 PWMGenerators::MCPWM::ePinPullMode::FLOATING);
    Actuators::Servo<PWMGenerators::MCPWM> _panServo
        = Actuators::Servo<PWMGenerators::MCPWM>(GET_SERVO_TIMING_CONFIG<eServoType::PAN>(),
                                                 __panServoPwmGen,
                                                 true,
                                                 static_cast<float>(DEG_TO_RAD) * 180.0F);

    LoopTimer<uint64_t, Time::millis> _timerPTZStatusSend;
};

#endif  // YAW_CONTROLLER_HPP
