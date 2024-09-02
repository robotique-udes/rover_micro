#include "Arduino.h"

#include "config_local.hpp"

#include "rover_can_lib/can_bus_manager.hpp"
#include "rover_can_lib/msgs/arm_cmd.hpp"
#include "rover_can_lib/msgs/arm_status.hpp"

#include "rover_helpers/helpers.hpp"
#include "systems/joint/dc_revolute_joint.hpp"
#include "actuators/motor_drivers/IFX007T.hpp"
#include "actuators/motor_drivers/DRV8251A.hpp"
#include "sensors/encoders/CUI_AMT222.hpp"
#include "rover_helpers/PID.hpp"
#include "sensors/limit_switch.hpp"
#include "SPI.h"

void CB_Can(RoverCanLib::CanBusManager *canManager_, const twai_message_t *msgPtr_);

float g_goalPosJ0 = 0.0f;
float g_goalPosJ1 = 0.0f;

void setup()
{
    Serial.begin(115200);
    LOG(WARN, "Init done starting!");

    DRV8251A motorJ0(PIN_J0_IN_1, PIN_J0_IN_2, MotorDriver::eBrakeMode::BRAKE, false, LEDC_TIMER_0, LEDC_CHANNEL_0, LEDC_CHANNEL_1);
    motorJ0.init();
    motorJ0.enable();
    motorJ0.setCmd(0.0f);
    motorJ0.setMaxVoltage(25.2f, 12.0f, false);

    // SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, GPIO_NUM_NC);
    // CUI_AMT222 encoderJ0(&SPI, PIN_SPI_CS_ENC_J0, true, Encoder::eEncoderType::ABSOLUTE_SINGLE_TURN);
    // encoderJ0.init();

    // PID pidPosJ0(100.0f, 10.0f, 10.0f, 10.0f);
    // DcRevoluteJoint j0(&motorJ0,
    //                    Encoder::eEncoderType::ABSOLUTE_SINGLE_TURN,
    //                    &encoderJ0,
    //                    Joint::eControlMode::POSITION,
    //                    false,
    //                    &pidPosJ0,
    //                    NULL);
    // j0.init();
    // j0.update();
    // j0.setPosition(j0.getPosition());
    // j0.setPIDDeadband(0.05f, 0.0f);

    IFX007T motorJ1(PIN_J1_EN_1, PIN_J1_EN_2, PIN_J1_IN_1, PIN_J1_IN_2, MotorDriver::eBrakeMode::BRAKE, true, LEDC_TIMER_0, LEDC_CHANNEL_2, LEDC_CHANNEL_3);
    motorJ1.init();
    motorJ1.enable();
    motorJ1.setCmd(0.0f);
    motorJ1.setMaxVoltage(25.2f, 25.2f, true);

    // SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, GPIO_NUM_NC);
    // CUI_AMT222 encoder(&SPI, PIN_SPI_CS_ENC_J1, true, Encoder::eEncoderType::ABSOLUTE_SINGLE_TURN);
    // encoder.init();

    // PID pidPosJ1(1000.0f, 10.0f, 10.0f, 5.0f);

    // DcRevoluteJoint j1(&motorJ1,
    //                    Encoder::eEncoderType::ABSOLUTE_SINGLE_TURN,
    //                    &encoder,
    //                    Joint::eControlMode::POSITION,
    //                    false,
    //                    &pidPosJ1,
    //                    NULL);
    // j1.init();
    // j1.update();
    // g_goalPosJ1 = j1.getPosition();
    // j1.setPosition(g_goalPosJ1);

    // j1.setPIDDeadband(0.005f, 0.0f);

    LimitSwitch switchFWDJ0(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_FWD_J0);
    switchFWDJ0.init();
    LimitSwitch switchREVJ0(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_REV_J0);
    switchREVJ0.init();

    LimitSwitch switchFWDJ1(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_FWD_J1);
    switchFWDJ1.init();
    LimitSwitch switchREVJ1(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_REV_J1);
    switchREVJ1.init();

    LimitSwitch switchCalib(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_CALIB);
    switchCalib.init();

    RoverCanLib::CanBusManager canBus(DEVICE_ID, PIN_RXC, PIN_TXC, CB_Can, false);
    canBus.init();

    RoverHelpers::Timer<unsigned long, millis> timerFeedback(50);
    RoverHelpers::Timer<unsigned long, millis> timer(500);

    for (;;)
    {
        canBus.update();

        if (g_goalPosJ0 > 0.1f)
        {
            motorJ0.setCmd(100.0f);
        }
        else if (g_goalPosJ0 < -0.1f)
        {
            motorJ0.setCmd(-100.0f);
        }
        else
        {
            motorJ0.setCmd(0.0f);
        }

        if (IN_ERROR(g_goalPosJ1, 10.0f, 0.0))
        {
            motorJ1.setCmd(0.0f);
        }
        else
        {
            motorJ1.setCmd(constrain(g_goalPosJ1, -100.0f, 100.0f));
        }

        if (switchFWDJ0.isClicked())
        {
            motorJ0.setCmd(100.0f);
        }
        else if (switchREVJ0.isClicked())
        {
            motorJ0.setCmd(-100.0f);
        }

        if (switchFWDJ1.isClicked())
        {
            motorJ1.setCmd(50.0f);
        }
        else if (switchREVJ1.isClicked())
        {
            motorJ1.setCmd(-50.0f);
        }

        if (timerFeedback.isDone())
        {
            RoverCanLib::Msgs::armStatus msg;
            msg.data.currentSpeed = 0.0f;
            canBus.sendMsg(&msg, (uint32_t)RoverCanLib::Constant::eDeviceId::J0_CONTROLLER);

            msg.data.currentSpeed = 0.0f;
            canBus.sendMsg(&msg, (uint32_t)RoverCanLib::Constant::eDeviceId::J1_CONTROLLER);
        }
    }
}

void loop() {}

void CB_Can(RoverCanLib::CanBusManager *canManager_, const twai_message_t *msgPtr_)
{
    if (msgPtr_->identifier == (uint32_t)RoverCanLib::Constant::eDeviceId::J0_CONTROLLER)
    {
        canManager_->resetWatchDog();
        RoverCanLib::Msgs::armCmd armMsg;
        canManager_->sendErrorCode(armMsg.parseMsg(msgPtr_));
        g_goalPosJ0 = armMsg.data.targetSpeed*1000.0f;
    }
    else if (msgPtr_->identifier == (uint32_t)RoverCanLib::Constant::eDeviceId::J1_CONTROLLER)
    {
        canManager_->resetWatchDog();
        RoverCanLib::Msgs::armCmd armMsg;
        canManager_->sendErrorCode(armMsg.parseMsg(msgPtr_));
        
        g_goalPosJ1 = armMsg.data.targetSpeed;
    }
}
