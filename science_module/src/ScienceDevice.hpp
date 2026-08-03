#ifndef SCIENCE_DEVICE_HPP
#define SCIENCE_DEVICE_HPP

#include "LinActuator.hpp"
#include "ServoController.hpp"
#include "config.hpp"
#include "rover_can2/device.hpp"
#include "rover_lib2/sensors/push_button.hpp"

#include "rover_can2/msgs/science_cmd.hpp"
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

    using DeviceT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::ScienceCmd, ScienceDevice>>;

  public:
    ScienceDevice() = default;

    void init()
    {
        this->_linAct.init();
        this->_linAct.setSpeed(FULL_STOP_SPEED);
        this->_servoCtrl.init();
    }

    void update()
    {
        if (!_loopTimer.isReady())
        {
            return;
        }

        this->_linAct.update();
        this->_servoCtrl.update();

        if (_pbUp.isClicked())
        {
            this->_linAct.setSpeed(JOG_SPEED);
        }
        else if (this->_pbDown.isClicked())
        {
            this->_linAct.setSpeed(-JOG_SPEED);
        }
        else if (this->_canWatchdog.isOk() && !IN_ERROR(_linActTargetSpeed, 0.001F, 0.0F))
        {
            this->_linAct.setSpeed(_linActTargetSpeed);
        }
        else
        {
            this->_linAct.setSpeed(FULL_STOP_SPEED);
        }

        if (this->_pbGrinder.isClicked() || (this->_canWatchdog.isOk() && this->_grinderOn))
        {
            this->_grinder.write(IO::eIOState::HIGH_);
        }
        else
        {
            this->_grinder.write(IO::eIOState::LOW_);
        }

        if (this->_pbCarroussel.isClicked() || (this->_canWatchdog.isOk() && this->_carrouselOn))
        {
            this->_servoCtrl.nextPosCarrousel();
        }

        if (_pbVacuum.isClicked())
        {
            if (this->_currentBeakPosition >= 180)
            {
                this->_currentBeakPosition = 0.0F;
            }
            else
            {
                this->_currentBeakPosition += 5.0F;
            }

            this->_servoCtrl.setPosition(this->_currentBeakPosition, eServoType::BEAK);
        }
        else if (this->_canWatchdog.isOk() && !IN_ERROR(this->_beakPos, 0.001F, 0.0F))
        {
            this->_servoCtrl.setBeakPositionFromCAN(this->_beakPos);
        }
        else
        {
            this->_servoCtrl.setPosition(0.0F, eServoType::BEAK);
        }
    }

    DeviceT& getUnderlyingCanDevice()
    {
        return this->_scienceCanDevice;
    }

  private:
    void CB_ScienceCmd(const RoverCan2::Msgs::ScienceCmd& msg_)
    {
        this->_canWatchdog.reset();
        this->_linActTargetSpeed = msg_.getData().lin_act_speed;
        this->_grinderOn = msg_.getData().grinder_on;
        this->_beakPos = msg_.getData().beak_pos;
        this->_carrouselOn = msg_.getData().carrousel_on;
    }

    LoopTimer<uint64_t, &Time::micros> _loopTimer = {LOOP_PERIOD_US};
    LinearAct _linAct;

    ServoController _servoCtrl;
    float _currentBeakPosition = 0.0F;

    PushButton _pbUp = {PIN_PB_UP};
    PushButton _pbDown = {PIN_PB_DOWN};
    PushButton _pbGrinder = {PIN_PB_GRINDER};
    PushButton _pbCarroussel = {PIN_PB_CARROUSSEL};
    PushButton _pbVacuum = {PIN_PB_VACUUM};

    IO::DigitalOutput _grinder = IO::DigitalOutput(PIN_GRINDER_PWM);

    float _linActTargetSpeed = 0.0F;
    bool _grinderOn = false;
    float _beakPos = false;
    bool _carrouselOn = false;

    Watchdog<uint64_t, &Time::millis> _canWatchdog = {CAN_WATCHDOG_VALIDITY_PERIOD};

    LoopTimer<uint64_t, &Time::millis> _timerCanSend = {CAN_SEND_PERIOD_MS};

    DeviceT _scienceCanDevice
        = DeviceT(RoverCan2::Constant::eDeviceId::SCIENCE,
                  RoverCan2::SubscriberMember<RoverCan2::Msgs::ScienceCmd, ScienceDevice>(*this, &ScienceDevice::CB_ScienceCmd));

    VALIDATE_CONCEPT(RoverObject, ScienceDevice);
};

#endif  // J34_DEVICE_HPP
