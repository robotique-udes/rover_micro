#ifndef J2_CONTROLLER_H
#define J2_CONTROLLER_H

#include <Arduino.h>
#include "rover_helpers/log.hpp"

/**
 * @brief 
 * @attention Begin must be called on serial 
 * 
 */
class J2Controller 
{
public:
    Stream* motorSerial; // Declare motorSerial as a reference to HardwareSerial

    // Motor Parameters
    float   param_mosTemp;
    float   param_motorTemp;
    float   param_outputCurrent;
    float   param_inputCurrent;
    float   param_idCurrent;
    float   param_iqCurrent;
    float   param_throttleValue;
    float   param_motorSpeed;
    float   param_inputVoltage;
    uint8_t param_motorStatusCode;
    float   param_motorOuterLoopPosition;
    uint8_t param_motorIdNumber;
    float   param_vdVoltage;
    float   param_vqVoltage;

private:
    // Checksum hash table
    static const unsigned short CRC16_TAB[]; // fuck le constexpr, faudrait que je déclare mon giga tableau ici, ça serait laid pas mal

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
    static constexpr uint8_t FRAME_HEAD = 0x02;
    static constexpr uint8_t FRAME_TAIL = 0x03;
    static constexpr uint8_t COMMAND_SET_RPM = 0x08;

    // Motor limtis
    static constexpr uint32_t RATED_SPEED_ERPM = 19572;
    static constexpr uint32_t MAX_SPEED_ERPM = 26880; //no load

    // Speed set/get
    float speedNowRadS;
    float speedNowRPM;
    float newSpeedRads;

public:
    J2Controller(Stream *serial_);
    void sendSpeedCommand(int32_t rpm_);
    unsigned short calculateCRC16(unsigned char *buf_, unsigned int len_);
    void readMotorParameters(bool verbose = false);
    void setSpeed(float newSpeedRadS_);
    float getSpeed(void);
    void update(void);
    ~J2Controller();
};

#endif // MOTOR_CONTROLLER_H