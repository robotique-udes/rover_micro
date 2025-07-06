#ifndef J2_CONTROLLER_H
#define J2_CONTROLLER_H

#include <Arduino.h>
#include "rover_lib2/helpers/log.hpp"

/**
 * @brief
 * @attention Begin must be called on serial
 *
 */
class J2Controller
{
  public:
    Stream* motorSerial;  // Declare motorSerial as a reference to HardwareSerial

    // Motor Parameters
    float param_mosTemp;
    float param_motorTemp;
    float param_outputCurrent;
    float param_inputCurrent;
    float param_idCurrent;
    float param_iqCurrent;
    float param_throttleValue;
    float param_motorSpeed;
    float param_inputVoltage;
    uint8_t param_motorStatusCode;
    float param_motorOuterLoopPosition;
    uint8_t param_motorIdNumber;
    float param_vdVoltage;
    float param_vqVoltage;
    uint32_t param_currentControlMode;
    float param_encoderAngle;
    float param_outerEncoderAngle;

  private:
    // Checksum hash table
    static const unsigned short
        CRC16_TAB[];  // fuck le constexpr, faudrait que je déclare mon giga tableau ici, ça serait laid pas mal

    // Simple I/O
    static constexpr uint8_t LED_ERR = 8;
    static constexpr uint8_t LED_BTLN = 9;
    static constexpr uint8_t BUT_JOG_PLUS = 10;
    static constexpr uint8_t BUT_JOG_MOINS = 11;

    // Motor UART config
    static constexpr uint8_t UART_TX_PIN = 6;
    static constexpr uint8_t UART_RX_PIN = 7;
    static constexpr uint32_t UART_BAUD_RATE = 921600;

    // Motor Protocol Constants
    static constexpr uint8_t FRAME_HEAD = 0xAA;
    static constexpr uint8_t FRAME_TAIL = 0xBB;
    static constexpr uint8_t COMMAND_SET_RPM = 0x49;
    static constexpr uint8_t COMMAND_GET_VALUES = 0x45;

    // Motor limtis
    static constexpr int32_t RATED_SPEED_ERPM = 19572;
    static constexpr int32_t MAX_SPEED_ERPM = 26880;  // no load

    // Ramp variable
    float current_rpm = 0.0f;
    float target_rpm = 0.0f;
    float ramp_rate = 1.0f;  // RPM per second
    uint32_t last_ramp_time = 0;

  public:
    J2Controller(Stream* serial_);
    void sendSpeedCommand(float rpm_);
    unsigned short calculateCRC16(unsigned char* buf_, unsigned int len_);
    void readMotorParameters(bool verbose = false);
    void setSpeed(float rpm_);
    float getSpeed(void);
    void update(void);
    bool isJogButtonPressed(bool plus_moins_);
    ~J2Controller();
};

#endif  // MOTOR_CONTROLLER_H