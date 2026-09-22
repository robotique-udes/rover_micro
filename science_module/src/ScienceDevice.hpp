#ifndef SCIENCE_DEVICE_HPP
#define SCIENCE_DEVICE_HPP

#include "config.hpp"
#include <Arduino.h>
#include <algorithm>

#include "LinActuator.hpp"
#include "ServoController.hpp"

#include "rover_lib2/sensors/K30.hpp"
#include "rover_lib2/sensors/push_button.hpp"

#include "rover_can2/device.hpp"
#include "rover_can2/msgs/science_cmd.hpp"
#include "rover_can2/msgs/science_info.hpp"
#include "rover_can2/rover_can2.hpp"

DEFINE_LOG_NODE(ScienceDevice, Logger::eNodeState::OFF);

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

    static constexpr uint8_t DEFAULT_SENSOR_ADDRESS = 0x68;
    static constexpr uint8_t SENSOR_1_ADDRESS = 0x68;
    static constexpr uint8_t SENSOR_2_ADDRESS = 0x69;
    static constexpr uint8_t SENSOR_3_ADDRESS = 0x70;
    static constexpr uint8_t ALL_SENSOR_ADDRESS = 0x7F;

    static constexpr uint16_t WET_VALUE = 3000;
    static constexpr uint16_t DRY_VALUE = 1000;

    static constexpr float HOME_POS_RAD = 0.85F;
    static constexpr float POUR_POS_RAD = 1.9F;
    static constexpr float DUMP_POS_RAD = std::numbers::pi_v<float>;

    enum class eServoPos : uint8_t
    {
        HOME = 0,
        POUR,
        DUMP,
    };

    using DeviceT = RoverCan2::Device<RoverCan2::SubscriberMember<RoverCan2::Msgs::ScienceCmd, ScienceDevice>,
                                      RoverCan2::Publisher<RoverCan2::Msgs::ScienceInfo>>;

  public:
    ScienceDevice() = default;
    LoopTimer<uint64_t, &Time::millis> timer1Hz = {1000ULL};
    void init()
    {
        Wire.setPins(PIN_I2C_SDA, PIN_I2C_SCL);
        Wire.begin();
        this->_linAct.init();
        this->_linAct.setSpeed(FULL_STOP_SPEED);
        this->_servoCtrl.init();
        this->_sense1.init();
        this->_sense2.init();
        this->_sense3.init();

        this->_grinder.write(IO::eIOState::LOW_);
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
        else if (this->_canWatchdog.isOk() && !IN_ERROR(_linActTargetSpeed, FULL_STOP_SPEED_ERROR_TOLERANCE, 0.0F))
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

        bool isCarrouselActive = this->_pbCarroussel.isClicked() || (this->_canWatchdog.isOk() && this->_carrouselOn);
        if (isCarrouselActive)
        {
            if (!this->_wasCarrouselOn)
            {
                this->_servoCtrl.nextPosCarrousel();
                this->_wasCarrouselOn = true;
            }
        }
        else
        {
            this->_wasCarrouselOn = false;
        }

        if (_pbBeak.isClicked())
        {
            this->_servoCtrl.setPosition(POUR_POS_RAD, eServoType::BEAK);
        }
        else if (this->_canWatchdog.isOk() && _beakPos == eServoPos::POUR)
        {
            this->_servoCtrl.setPosition(POUR_POS_RAD, eServoType::BEAK);
        }
        else if (this->_canWatchdog.isOk() && _beakPos == eServoPos::DUMP)
        {
            this->_servoCtrl.setPosition(DUMP_POS_RAD, eServoType::BEAK);
        }
        else
        {
            this->_servoCtrl.setPosition(HOME_POS_RAD, eServoType::BEAK);
        }

        if (_timerCanSend.isReady())
        {
            RoverCan2::Msgs::ScienceInfo infoMsg;

            infoMsg.data().sample_index = this->_sampleIndex++;
            infoMsg.data().sensor_1 = this->_sense1.getCO2();
            infoMsg.data().sensor_2 = this->_sense2.getCO2();
            infoMsg.data().sensor_3 = this->_sense3.getCO2();
            // infoMsg.data().humidity = readAveraged(PIN_SERVO_2);
            infoMsg.data().humidity = 0;

            this->_canDevice.sendMsg(infoMsg);
        }
    }

    DeviceT& getUnderlyingCanDevice()
    {
        return this->_canDevice;
    }

  private:
    void CB_ScienceCmd(const RoverCan2::Msgs::ScienceCmd& msg_)
    {
        this->_canWatchdog.reset();
        this->_linActTargetSpeed = msg_.getData().lin_act_speed;
        this->_grinderOn = msg_.getData().grinder_on;
        this->_beakPos = static_cast<eServoPos>(msg_.getData().beak_pos);
        this->_carrouselOn = msg_.getData().carrousel_on;
    }

    int readAveraged(int pin, int samples = 16)
    {
        long sum = 0;
        for (int i = 0; i < samples; i++)
        {
            sum += analogRead(pin);
            delayMicroseconds(200);
        }
        return sum / samples;
    }

    float readMoisturePercent(int pin)
    {
        int raw = readAveraged(pin);
        raw = constrain(raw, DRY_VALUE, WET_VALUE);
        return 100.0 * (DRY_VALUE - raw) / (float)(DRY_VALUE - WET_VALUE);
    }

    LoopTimer<uint64_t, &Time::micros> _loopTimer = {LOOP_PERIOD_US};
    LinearAct _linAct;

    ServoController _servoCtrl;
    float _currentBeakPosition = 0.0F;
    bool _wasCarrouselOn = false;

    PushButton _pbUp = {PIN_PB_UP, PushButton::ePullMode::PULL_UP, GPIO_PULLUP_ONLY};
    PushButton _pbDown = {PIN_PB_DOWN, PushButton::ePullMode::PULL_UP, GPIO_PULLUP_ONLY};
    PushButton _pbGrinder = {PIN_PB_GRINDER, PushButton::ePullMode::PULL_UP, GPIO_PULLUP_ONLY};
    PushButton _pbCarroussel = {PIN_PB_CARROUSSEL, PushButton::ePullMode::PULL_UP, GPIO_PULLUP_ONLY};
    PushButton _pbBeak = {PIN_PB_VACUUM, PushButton::ePullMode::PULL_UP, GPIO_PULLUP_ONLY};
    PushButton _pbSpare = {PIN_PB_SPARE, PushButton::ePullMode::PULL_UP, GPIO_PULLUP_ONLY};

    IO::DigitalOutput _grinder = IO::DigitalOutput(PIN_GRINDER_PWM);
    // IO::AnalogInput _humiditySensor = IO::DigitalInput(PIN_SERVO_2);

    float _linActTargetSpeed = 0.0F;
    bool _grinderOn = false;
    eServoPos _beakPos = eServoPos::HOME;
    bool _carrouselOn = false;

    K30 _sense1 = K30(Wire, SENSOR_1_ADDRESS);
    K30 _sense2 = K30(Wire, SENSOR_2_ADDRESS);
    K30 _sense3 = K30(Wire, SENSOR_3_ADDRESS);
    uint32_t _sampleIndex = 0U;

    Watchdog<uint64_t, &Time::millis> _canWatchdog = {CAN_WATCHDOG_VALIDITY_PERIOD};
    LoopTimer<uint64_t, &Time::millis> _timerCanSend = {CAN_SEND_PERIOD_MS};

    DeviceT _canDevice
        = DeviceT(RoverCan2::Constant::eDeviceId::SCIENCE,
                  RoverCan2::SubscriberMember<RoverCan2::Msgs::ScienceCmd, ScienceDevice>(*this, &ScienceDevice::CB_ScienceCmd),
                  RoverCan2::Publisher<RoverCan2::Msgs::ScienceInfo>());

    VALIDATE_CONCEPT(RoverObject, ScienceDevice);
};

#endif  // SCIENCE_DEVICE_HPP
