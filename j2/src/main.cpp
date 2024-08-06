#include "Arduino.h"

#include "config_local.hpp"

#include "rover_can_lib/can_bus_manager.hpp"
#include "rover_can_lib/msgs/arm_cmd.hpp"
#include "rover_can_lib/msgs/arm_status.hpp"

#include "rover_helpers/helpers.hpp"
#include "systems/joint/dc_revolute_joint.hpp"
#include "actuators/motor_drivers/IFX007T.hpp"
#include "sensors/encoders/CUI_AMT222.hpp"
#include "rover_helpers/PID.hpp"
#include "sensors/limit_switch.hpp"
#include "SPI.h"

void CB_Can(RoverCanLib::CanBusManager *canManager_, const twai_message_t *msgPtr_);

float g_goalPosJ2 = 0.0f;

void setup()
{
    Serial.begin(115200);
    delay(2000);

    RoverCanLib::CanBusManager canBus(DEVICE_ID, PIN_RXC, PIN_TXC, CB_Can, false);
    canBus.init();

    LOG(WARN, "Init done starting!");

    IFX007T motor(PIN_J2_EN_1, PIN_J2_EN_2, PIN_J2_IN_1, PIN_J2_IN_2, MotorDriver::eBrakeMode::BRAKE, false);
    motor.init();
    motor.enable();
    motor.setCmd(0.0f);
    motor.setMaxVoltage(25.2f, 25.2f, true);

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, GPIO_NUM_NC);
    CUI_AMT222 encoder(&SPI, PIN_SPI_CS_EN_SHAFT, false, Encoder::eEncoderType::ABSOLUTE_SINGLE_TURN);
    encoder.init();

#warning TODO: Tuning
    PID pidPos(600.0f, 20.0f, 100.0f, 80.0f);

    DcRevoluteJoint j2(&motor,
                       Encoder::eEncoderType::ABSOLUTE_SINGLE_TURN,
                       &encoder,
                       Joint::eControlMode::POSITION,
                       false,
                       &pidPos);
    // j2.setJointLimits(DEG_TO_RAD * -40.0f, DEG_TO_RAD * 40.0f);
    j2.init();

    j2.update();
    j2.setPosition(j2.getPosition());

    LimitSwitch switchFWD(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_FWD);
    switchFWD.init();
    LimitSwitch switchREV(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_REV);
    switchREV.init();

    LimitSwitch switchCalib(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_CALIB);
    switchCalib.init();

    RoverHelpers::Timer<unsigned long, millis> timerFeedback(50);
    RoverHelpers::Timer<unsigned long, millis> timer(500);

    for (;;)
    {
        canBus.update();
        j2.update();
        j2.printDebugInfo();

        if (canBus.isOk())
        {
            j2.setPosition(g_goalPosJ2);
        }
        else
        {
            j2.setPosition(j2.getPosition());
        }

        if (timerFeedback.isDone())
        {
            RoverCanLib::Msgs::armStatus msg;
            msg.data.currentSpeed = j2.getPosition();
            canBus.sendMsg(&msg);
        }
    }
}

void loop() {}

void CB_Can(RoverCanLib::CanBusManager *canManager_, const twai_message_t *msgPtr_)
{
    if (msgPtr_->identifier == (uint32_t)DEVICE_ID)
    {
        canManager_->resetWatchDog();
        RoverCanLib::Msgs::armCmd armMsg;
        canManager_->sendErrorCode(armMsg.parseMsg(msgPtr_));

        g_goalPosJ2 = armMsg.data.targetSpeed;
    }
}
