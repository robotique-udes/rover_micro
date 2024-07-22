#ifndef _WT901_HPP_
#define _WT901_HPP_

#include <Arduino.h>

#include "rover_helpers/log.hpp"

class WT901
{
private:
    static constexpr uint8_t BUFFER_SIZE = 11;
    uint8_t _bufferIndex = 0;
    uint8_t buffer[BUFFER_SIZE];

    gpio_num_t _RX_pin = GPIO_NUM_NC;
    gpio_num_t _TX_pin = GPIO_NUM_NC;

    float _roll;
    float _pitch;
    float _yaw;

    void parseAnglePacket()
    {
        int16_t roll = (int16_t)((buffer[3] << 8) | buffer[2]);
        int16_t pitch = (int16_t)((buffer[5] << 8) | buffer[4]);
        int16_t yaw = (int16_t)((buffer[7] << 8) | buffer[6]);
        uint8_t VL = buffer[8];
        uint8_t VH = buffer[9];
        uint8_t receivedSum = buffer[10];
        uint8_t sum = 0x55 + 0x53 + buffer[2] + buffer[3] + buffer[4] + buffer[5] + buffer[6] + buffer[7] + VL + VH;

        if (sum == receivedSum) {
            _roll = (roll / 32768.0) * 180.0;
            _pitch = (pitch / 32768.0) * 180.0;
            _yaw = (yaw / 32768.0) * 180.0;

            _yaw = fmod(_yaw + 360.0, 360.0);
            
            // FOR PYTHON VISUALIZATION SCRIPT
            
            Serial.print(_pitch);
            Serial.print(", ");
            Serial.print(_roll);
            Serial.print(", ");
            Serial.println(_yaw);
            
            
        } else {
            LOG(ERROR, "Checksum error in angle packet");
        }
    }

public:

    WT901(gpio_num_t RX_pin_,  gpio_num_t TX_pin_)
        : _RX_pin{RX_pin_},_TX_pin{TX_pin_} {}

    void init()
    {
        Serial1.begin(9600, SERIAL_8N1, _RX_pin, _TX_pin);
    }

    void updateOrientation()
    {
        while (Serial1.available()) 
        {
            uint8_t c = Serial1.read();
            buffer[_bufferIndex++] = c;

            if (_bufferIndex == BUFFER_SIZE) 
            {
                if (buffer[0] == 0x55 && buffer[1] == 0x53) 
                {
                    parseAnglePacket();
                }
                _bufferIndex = 0;
            }
        }
    }

    float getRoll() const { return _roll; }
    float getPitch() const { return _pitch; }
    float getYaw() const { return _yaw; }
};

#endif // _WT901_HPP_
