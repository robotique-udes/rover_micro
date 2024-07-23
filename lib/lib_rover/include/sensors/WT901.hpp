#ifndef _WT901_HPP_
#define _WT901_HPP_

#include <Arduino.h>

#include "rover_helpers/log.hpp"

class WT901
{
public:
    WT901(gpio_num_t RX_pin_, gpio_num_t TX_pin_)
        : _RX_pin{RX_pin_}, _TX_pin{TX_pin_} {}

    void init()
    {
        Serial1.begin(9600, SERIAL_8N1, _RX_pin, _TX_pin);
    }

    void updateOrientation()
    {
        while (Serial1.available())
        {
            uint8_t c = Serial1.read();
            _buffer[_bufferIndex++] = c;

            if (_bufferIndex == BUFFER_SIZE)
            {
                if (_buffer[0] == 0x55 && _buffer[1] == 0x53)
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

private:
    static constexpr uint8_t BUFFER_SIZE = 11;
    uint8_t _bufferIndex = 0;
    uint8_t _buffer[BUFFER_SIZE];

    gpio_num_t _RX_pin = GPIO_NUM_NC;
    gpio_num_t _TX_pin = GPIO_NUM_NC;

    float _roll = 0;
    float _pitch = 0;
    float _yaw = 0;

    void parseAnglePacket()
    {
        int16_t roll = (int16_t)((_buffer[3] << 8) | _buffer[2]);
        int16_t pitch = (int16_t)((_buffer[5] << 8) | _buffer[4]);
        int16_t yaw = (int16_t)((_buffer[7] << 8) | _buffer[6]);
        uint8_t VL = _buffer[8];
        uint8_t VH = _buffer[9];
        uint8_t receivedSum = _buffer[10];
        uint8_t sum = 0x55 + 0x53 + _buffer[2] + _buffer[3] + _buffer[4] + _buffer[5] + _buffer[6] + _buffer[7] + VL + VH;

        if (sum == receivedSum)
        {
            _roll = (roll / 32768.0) * 180.0;
            _pitch = (pitch / 32768.0) * 180.0;
            _yaw = (yaw / 32768.0) * 180.0;

            _yaw = fmod(_yaw + 360.0, 360.0);

            // FOR PYTHON VISUALIZATION SCRIPT
            // Serial.printf("%.2f, %.2f, %.2f\n", _pitch, _roll, _yaw);
        }
        else
        {
            LOG(ERROR, "Checksum error in angle packet");
        }
    }
};

#endif // _WT901_HPP_
