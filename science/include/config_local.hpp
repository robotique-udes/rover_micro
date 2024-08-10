#ifndef __CONFIG_LOCAL_HPP__
#define __CONFIG_LOCAL_HPP__

// This file should be use to change parameter before uploading into a new
// device of the same type. For example, all propulsion motors should have the
// same code other then the constant specified here.
#include <cstdint>
#include "Arduino.h"
#include "rover_can_lib/constant.hpp"

constexpr uint16_t DEVICE_ID = (uint16_t)RoverCanLib::Constant::eDeviceId::SCIENCE;

constexpr gpio_num_t LIN_IN_1 = GPIO_NUM_18;
constexpr gpio_num_t LIN_IN_2 = GPIO_NUM_8;

constexpr gpio_num_t LS_EXT = GPIO_NUM_1;
constexpr gpio_num_t LS_RET = GPIO_NUM_2;

constexpr gpio_num_t PB_GRINDER = GPIO_NUM_11;
constexpr gpio_num_t PB_CARROUSSEL = GPIO_NUM_12;
constexpr gpio_num_t PB_SPARE = GPIO_NUM_13;
constexpr gpio_num_t PB_UP = GPIO_NUM_14;
constexpr gpio_num_t PB_DOWN = GPIO_NUM_21;
constexpr gpio_num_t PB_VACCUM = GPIO_NUM_38;

constexpr gpio_num_t SERVO_0 = GPIO_NUM_7;
constexpr gpio_num_t SERVO_1 = GPIO_NUM_15;
constexpr gpio_num_t SERVO_2 = GPIO_NUM_16;
constexpr gpio_num_t SERVO_3 = GPIO_NUM_17;

constexpr gpio_num_t FAN_A_PWM = GPIO_NUM_9;
constexpr gpio_num_t FAN_B_PWM = GPIO_NUM_3;
constexpr gpio_num_t GRINDER_PWM = GPIO_NUM_10;

constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_47;
constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_48;

#endif // __CONFIG_LOCAL_HPP__
