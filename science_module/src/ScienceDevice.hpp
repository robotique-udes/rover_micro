#ifndef SCIENCE_DEVICE_HPP
#define SCIENCE_DEVICE_HPP

#include "LinActuator.hpp"
#include "ServoController.hpp"
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

    static constexpr float JOG_SPEED = -1000.0F;
    static constexpr float FULL_STOP_SPEED = 0.0F;
    static constexpr float CALIB_POSITION = 0.0F;

    static constexpr float FULL_STOP_SPEED_ERROR_TOLERANCE = 0.01F;  // m

    using DeviceT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::Science, ScienceDevice>>;

  public:
    ScienceDevice() = default;

    void init()
    {
        _linAct.init();
        _linAct.setSpeed(FULL_STOP_SPEED);
        _servoCtrl.init();
    }

    void update()
    {
        if (!_loopTimer.isReady())
        {
            return;
        }

        _linAct.update();
        _servoCtrl.update();

        if (_pbUp.isClicked())
        {
            _linAct.setSpeed(JOG_SPEED);
        }
        else if (_pbDown.isClicked())
        {
            _linAct.setSpeed(-JOG_SPEED);
        }
        else
        {
            _linAct.setSpeed(FULL_STOP_SPEED);
        }

        if (_pbGrinder.isClicked())
        {
            _grinder.write(IO::eIOState::HIGH_);
        }
        else
        {
            _grinder.write(IO::eIOState::LOW_);
        }

        if (_pbCarroussel.isClicked())
        {
            if (this->_currentCarrouselPosition >= 180.0F)
            {
                this->_currentCarrouselPosition = 0.0F;
            }
            else
            {
                this->_currentCarrouselPosition += 45.0F;
            }

            this->_servoCtrl.setPosition(this->_currentCarrouselPosition, eServoType::CARROUSEL);
        }

        if (_pbVacuum.isClicked())
        {
            if (this->_currentBeakPosition >= 300.0F)
            {
                this->_currentBeakPosition = 0.0F;
            }
            else
            {
                this->_currentBeakPosition += 5.0F;
            }

            this->_servoCtrl.setPosition(this->_currentBeakPosition, eServoType::BEAK);
        }
    }

    DeviceT& getUnderlyingCanDevice()
    {
        return _scienceCanDevice;
    }

  private:
    void CB_ScienceCmd(const RoverCan2::Msgs::Science& msg_)
    {
        _scienceCanWatchdog.reset();
        _linActTargetSpeed = msg_.getData().lin_act_speed;
    }

    LoopTimer<uint64_t, &Time::micros> _loopTimer = {LOOP_PERIOD_US};
    LinearAct _linAct;

    ServoController _servoCtrl;
    float _currentBeakPosition = 0.0F;
    float _currentCarrouselPosition = 0.0F;

    PushButton _pbUp = {PIN_PB_UP};
    PushButton _pbDown = {PIN_PB_DOWN};
    PushButton _pbGrinder = {PIN_PB_GRINDER};
    PushButton _pbCarroussel = {PIN_PB_CARROUSSEL};
    PushButton _pbVacuum = {PIN_PB_VACUUM};

    IO::DigitalOutput _grinder = IO::DigitalOutput(PIN_GRINDER_PWM);

    float _linActTargetSpeed = 0.0F;
    Watchdog<uint64_t, &Time::millis> _scienceCanWatchdog = {CAN_WATCHDOG_VALIDITY_PERIOD};

    LoopTimer<uint64_t, &Time::millis> _timerCanSend = {CAN_SEND_PERIOD_MS};

    DeviceT _scienceCanDevice
        = DeviceT(RoverCan2::Constant::eDeviceId::SCIENCE,
                  RoverCan2::SubscriberMember<RoverCan2::Msgs::Science, ScienceDevice>(*this, &ScienceDevice::CB_ScienceCmd));

    VALIDATE_CONCEPT(RoverObject, ScienceDevice);
};

#endif  // J34_DEVICE_HPP
