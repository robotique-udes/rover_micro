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
    // Simple I/O
    static constexpr uint8_t LED_ERR = 8;
    static constexpr uint8_t LED_BTLN = 9;
    static constexpr uint8_t BUT_JOG_PLUS = 10;
    static constexpr uint8_t BUT_JOG_MOINS = 11;

    // Motor UART config
    static constexpr uint8_t UART_TX_PIN = 6;
    static constexpr uint8_t UART_RX_PIN = 7;
    static constexpr uint32_t UART_BAUD_RATE = 921600;

    // Motor limtis
    static constexpr int32_t RATED_SPEED_ERPM = 19572;
    static constexpr int32_t MAX_SPEED_ERPM = 26880;  // no load

  private:
    // Checksum hash table (fuck le constexpr, faudrait que je déclare mon giga tableau ici, ça serait laid pas mal)
    static const unsigned short CRC16_TAB[];

    // Motor Protocol Constants
    static constexpr uint8_t FRAME_HEAD = 0xAA;
    static constexpr uint8_t FRAME_TAIL = 0xBB;
    static constexpr uint8_t COMMAND_SET_RPM = 0x49;
    static constexpr uint8_t COMMAND_GET_VALUES = 0x45;

  public:
    // Motor Parameters
    struct MotorParam
    {
        float mosTemp;
        float motorTemp;
        float outputCurrent;
        float inputCurrent;
        float idCurrent;
        float iqCurrent;
        float throttleValue;
        float motorSpeed;
        float inputVoltage;
        uint8_t motorStatusCode;
        float motorOuterLoopPosition;
        uint8_t motorIdNumber;
        float vdVoltage;
        float vqVoltage;
        uint32_t currentControlMode;
        float encoderAngle;
        float outerEncoderAngle;
    };
    MotorParam sMotorParam;

  private:
    Stream* _motorSerial;  // Declare motorSerial as a reference to HardwareSerial

    // Ramp variables
    float _currentRPM = 0.0f;
    float _targetRPM = 0.0f;
    float _rampRate = 8.0f;  // RPM per second
    uint32_t _lastRampTime = 0;

  public:
    J2Controller(Stream* serial_);
    void sendSpeedCommand(float rpm_);
    unsigned short calculateCRC16(unsigned char* buf_, unsigned int len_);
    void readMotorParameters(bool verbose_ = false);
    void setSpeed(float rpm_);
    float getSpeed(void);
    void update(void);
    bool isJogButtonPressed(bool plus_moins_);
    ~J2Controller();
};

#endif  // MOTOR_CONTROLLER_H