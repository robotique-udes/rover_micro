#include "Arduino.h"
#include "Preferences.h"
#include "config_local.hpp"

#include "rover_can_lib/can_bus_manager.hpp"
#include "rover_can_lib/msgs/arm_cmd.hpp"
#include "rover_can_lib/msgs/arm_status.hpp"

#include "rover_helpers/helpers.hpp"
#include "systems/joint/dc_revolute_joint.hpp"
#include "systems/joint/differential_joint.hpp"
#include "actuators/motor_drivers/DRV8251A.hpp"
#include "sensors/encoders/CUI_AMT222.hpp"
#include "rover_helpers/PID.hpp"
#include "sensors/limit_switch.hpp"

constexpr unsigned long TIME_CALIB_DEBOUNCE = 1000ul; // ms
constexpr char EEPROM_NS[] = {"ns"};
constexpr char EEPROM_JOINT_A_OFFSET[] = {"ja"};
constexpr char EEPROM_JOINT_B_OFFSET[] = {"jb"};

constexpr float GRIP_GOAL_THRESHOLD = 0.9f;  // 90%
constexpr float GRIPPER_MOTOR_SPEED = 50.0f; // % Percent of max cmd

constexpr uint8_t ADDRESS_CALIB_DIFF_A = 0x10;
constexpr uint8_t ADDRESS_CALIB_DIFF_B = 0x20;

void saveCalibOffset(Preferences *eeprom, Joint *jA_, Joint *jB_);
void applyCalibOffset(Preferences *eeprom, Joint *jA_, Encoder *jAEnc_, Joint *jB_, Encoder *jBEnc_);

void CB_Can(RoverCanLib::CanBusManager *canManager_, const twai_message_t *msgPtr_);

float g_posDiffUpDown = 0.0f;
float g_posDiffRot = 0.0f;
float g_posGrip = 0.0f;

void setup()
{
    Preferences eeprom;

    Serial.begin(115200);
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, GPIO_NUM_NC);

    RoverCanLib::CanBusManager canBus(DEVICE_ID, PIN_RXC, PIN_TXC, CB_Can, false);
    canBus.init();

    // =========================================================================
    // Diff init
    // =========================================================================
    DRV8251A motorDiffA(PIN_DIFF_A_IN_1,
                        PIN_DIFF_A_IN_2,
                        MotorDriver::eBrakeMode::BRAKE,
                        false,
                        LEDC_TIMER_0,
                        LEDC_CHANNEL_0,
                        LEDC_CHANNEL_1);
    motorDiffA.setMaxVoltage(25.2f, 5.0f, false);
    CUI_AMT222 encoderDiffA(&SPI, PIN_CS_DIFF_A, false, Encoder::eEncoderType::ABSOLUTE_MULTI_TURN);
    PID pidA(100'000.0f, 4.75f, 0.0f, 50.0f);

    DcRevoluteJoint jointDiffA(&motorDiffA,
                               Encoder::eEncoderType::ABSOLUTE_MULTI_TURN,
                               &encoderDiffA,
                               Joint::eControlMode::POSITION,
                               false,
                               &pidA);
    jointDiffA.setPIDDeadband(DEG_TO_RAD * 0.5f, Joint::DEFAULT_PID_DEADBAND_RAD_SPEED);

    DRV8251A motorDiffB(PIN_DIFF_B_IN_1,
                        PIN_DIFF_B_IN_2,
                        MotorDriver::eBrakeMode::BRAKE,
                        false,
                        LEDC_TIMER_0,
                        LEDC_CHANNEL_2,
                        LEDC_CHANNEL_3);
    motorDiffB.setMaxVoltage(25.2f, 5.0f, false);
    CUI_AMT222 encoderDiffB(&SPI, PIN_CS_DIFF_B, true, Encoder::eEncoderType::ABSOLUTE_MULTI_TURN);
    PID pidB(100'000.0f, 4.75f, 0.0f, 50.0f);
    DcRevoluteJoint jointDiffB(&motorDiffB,
                               Encoder::eEncoderType::ABSOLUTE_MULTI_TURN,
                               &encoderDiffB,
                               Joint::eControlMode::POSITION,
                               false,
                               &pidB);
    jointDiffB.setPIDDeadband(DEG_TO_RAD * 0.5f, Joint::DEFAULT_PID_DEADBAND_RAD_SPEED);

    DifferentialJoint differential(&jointDiffA, &jointDiffB, Joint::eControlMode::POSITION);
    differential.init();
    differential.setJointLimitsUpDown(DEG_TO_RAD * -110.0f, DEG_TO_RAD * 81.0f);
    differential.setJointLimitsRot(DEG_TO_RAD * -70.0f, DEG_TO_RAD * 110.0f);

    // =========================================================================
    // Gripper Init
    // =========================================================================
    DRV8251A motorGripper(PIN_GRIP_IN_1,
                          PIN_GRIP_IN_2,
                          MotorDriver::eBrakeMode::BRAKE,
                          false,
                          LEDC_TIMER_0,
                          LEDC_CHANNEL_4,
                          LEDC_CHANNEL_5);
    motorGripper.init();
    motorGripper.enable();
    LimitSwitch SW_GripClosed(LimitSwitch::eLimitSwitchMode::PullUp, PIN_SWT_A_GRIP);
    SW_GripClosed.init();
    LimitSwitch SW_GripOpenned(LimitSwitch::eLimitSwitchMode::PullUp, PIN_SWT_B_GRIP);
    SW_GripClosed.init();
    RoverHelpers::MovingAverage<bool, 100> avgGripIsClosed = RoverHelpers::MovingAverage<bool, 100>(SW_GripClosed.isClicked());
    RoverHelpers::MovingAverage<bool, 100> avgGripIsOpenned = RoverHelpers::MovingAverage<bool, 100>(SW_GripOpenned.isClicked());
    RoverHelpers::Timer<unsigned long, millis> timerGripCheck = RoverHelpers::Timer<unsigned long, millis>(5); // 450 ms of overshoot

    // =========================================================================
    // Jog switch init
    // =========================================================================
    LimitSwitch SW_diffUp(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_DIFF_UP);
    SW_diffUp.init();
    LimitSwitch SW_diffDown(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_DIFF_DOWN);
    SW_diffDown.init();
    LimitSwitch SW_diffRotL(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_DIFF_LEFT);
    SW_diffRotL.init();
    LimitSwitch SW_diffRotR(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_DIFF_RIGHT);
    SW_diffRotR.init();
    LimitSwitch SW_gripClose(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_GRIP_FWD);
    SW_gripClose.init();
    LimitSwitch SW_gripOpen(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_GRIP_REV);
    SW_gripOpen.init();

    // =========================================================================
    // Start of sequence
    // =========================================================================
    applyCalibOffset(&eeprom, &jointDiffA, &encoderDiffA, &jointDiffB, &encoderDiffB);

    LOG(WARN, "Init done starting!");
    RoverHelpers::Timer<unsigned long, millis> timerFeedback(100); // 10 Hz

    differential.update();
    g_posDiffUpDown = differential.getPositionUpDown();
    g_posDiffRot = differential.getPositionRot();
    differential.setPosition(g_posDiffUpDown, g_posDiffRot);

    for (;;)
    {
        canBus.update();
        differential.update();

        if (timerGripCheck.isDone())
        {
            if (g_posGrip > 0.0f) // Closed
            {
                if (avgGripIsClosed.addValue(SW_GripClosed.isClicked()) > GRIP_GOAL_THRESHOLD)
                {
                    motorGripper.setCmd(0.0f);
                }
                else
                {
                    motorGripper.setCmd(GRIPPER_MOTOR_SPEED);
                }
            }
            else // openned
            {
                if (avgGripIsOpenned.addValue(SW_GripOpenned.isClicked()) > GRIP_GOAL_THRESHOLD)
                {
                    motorGripper.setCmd(0.0f);
                }
                else
                {
                    motorGripper.setCmd(-GRIPPER_MOTOR_SPEED);
                }
            }
        }

#warning TODO
        if (canBus.isOk() /* && watchdog ok TODO*/)
        {
            differential.setPosition(g_posDiffUpDown, g_posDiffRot);
        }
        else
        {
            differential.setPosition(differential.getPositionUpDown(), differential.getPositionRot());
        }

        // Missing a calib button, must only calib at 0 rot and 0 up/down
        if (SW_gripClose.isClicked() && SW_gripOpen.isClicked())
        {
            saveCalibOffset(&eeprom, &jointDiffA, &jointDiffB);
            applyCalibOffset(&eeprom, &jointDiffA, &encoderDiffA, &jointDiffB, &encoderDiffB);
        }
        else if (SW_diffUp.isClicked())
        {
            differential.setPosition(differential.getPositionUpDown() + 0.01f, differential.getPositionRot());
        }
        else if (SW_diffDown.isClicked())
        {
            differential.setPosition(differential.getPositionUpDown() - 0.01f, differential.getPositionRot());
        }
        else if (SW_diffRotL.isClicked())
        {
            differential.setPosition(differential.getPositionUpDown(), differential.getPositionRot() + 0.02f);
        }
        else if (SW_diffRotR.isClicked())
        {
            differential.setPosition(differential.getPositionUpDown(), differential.getPositionRot() - 0.02f);
        }

        if (timerFeedback.isDone())
        {
            RoverCanLib::Msgs::armStatus msg;
            msg.data.currentSpeed = differential.getPositionUpDown();
            canBus.sendMsg(&msg, (uint32_t)RoverCanLib::Constant::eDeviceId::GRIPPER_TILT_CONTROLLER);

            msg.data.currentSpeed = differential.getPositionRot();
            canBus.sendMsg(&msg, (uint32_t)RoverCanLib::Constant::eDeviceId::GRIPPER_ROT_CONTROLLER);

            // RoverCanLib::Msgs::armStatus msg;
            // msg.data.currentSpeed = differential.getPositionRot();
            // canBus.sendMsg(&msg, (uint32_t)RoverCanLib::Constant::eDeviceId::J5_CONTROLLER);
        }
    }
}

void loop() {}

void applyCalibOffset(Preferences *eeprom, Joint *jA_, Encoder *jAEnc_, Joint *jB_, Encoder *jBEnc_)
{
    eeprom->begin(EEPROM_NS, true);

    if (eeprom->isKey(EEPROM_JOINT_A_OFFSET) && eeprom->isKey(EEPROM_JOINT_B_OFFSET))
    {
        float positionOffsetA = -eeprom->getFloat(EEPROM_JOINT_A_OFFSET);
        float positionOffsetB = -eeprom->getFloat(EEPROM_JOINT_B_OFFSET);

        jAEnc_->calib(positionOffsetA, true);
        jBEnc_->calib(positionOffsetB, true);

        jA_->update();
        jB_->update();

        jA_->setPosition(jA_->getPosition());
        jB_->setPosition(jB_->getPosition());

        RoverHelpers::Chrono<unsigned long, millis> debounceTime;
        while (debounceTime.getTime() < TIME_CALIB_DEBOUNCE)
        {
            jA_->update();
            jB_->update();
        }
    }
    else
    {
        LOG(ERROR, "Couldn't get calib values from EEPROM, %i, %i", eeprom->isKey(EEPROM_JOINT_A_OFFSET), eeprom->isKey(EEPROM_JOINT_B_OFFSET));
    }

    eeprom->end();
}

void saveCalibOffset(Preferences *eeprom, Joint *jA_, Joint *jB_)
{
    eeprom->begin(EEPROM_NS, false);
    float positionOffsetA = -jA_->getPosition(true);
    float positionOffsetB = -jB_->getPosition(true);

    LOG(DEBUG, "Nb of bytes written: %u", eeprom->putFloat(EEPROM_JOINT_A_OFFSET, positionOffsetA));
    LOG(DEBUG, "Nb of bytes written: %u", eeprom->putFloat(EEPROM_JOINT_B_OFFSET, positionOffsetB));

    eeprom->end();
}

void CB_Can(RoverCanLib::CanBusManager *canManager_, const twai_message_t *msgPtr_)
{
    switch ((RoverCanLib::Constant::eDeviceId)msgPtr_->identifier)
    {
    case (RoverCanLib::Constant::eDeviceId::GRIPPER_TILT_CONTROLLER):
    {
        RoverCanLib::Msgs::armCmd armMsg;
        canManager_->sendErrorCode(armMsg.parseMsg(msgPtr_));

        g_posDiffUpDown = armMsg.data.targetSpeed;
        LOG(INFO, "Received targetPos: %f", g_posDiffUpDown);
        break;
    }
    case (RoverCanLib::Constant::eDeviceId::GRIPPER_ROT_CONTROLLER):
    {
        RoverCanLib::Msgs::armCmd armMsg;
        canManager_->sendErrorCode(armMsg.parseMsg(msgPtr_));

        g_posDiffRot = armMsg.data.targetSpeed;
        break;
    }
    case (RoverCanLib::Constant::eDeviceId::GRIPPER_CLOSE_CONTROLLER):
    {
        RoverCanLib::Msgs::armCmd armMsg;
        canManager_->sendErrorCode(armMsg.parseMsg(msgPtr_));

        g_posGrip = armMsg.data.targetSpeed;
        break;
    }
    default:
        // Device not concerned by msg
        break;
    }
}
