#ifndef __CONFIG_LOCAL_HPP__
#define __CONFIG_LOCAL_HPP__

// This file should be use to change parameter before uploading into a new
// device of the same type. For example, all propulsion motors should have the
// same code other then the constant specified here.
#include <cstdint>
#include "Arduino.h"
#include "rover_can_lib/constant.hpp"

constexpr uint16_t DEVICE_ID = (uint16_t)RoverCanLib::Constant::eDeviceId::GRIPPER_ROT_CONTROLLER;

constexpr gpio_num_t PIN_DIFF_A_IN_1 = GPIO_NUM_38;
constexpr gpio_num_t PIN_DIFF_A_IN_2 = GPIO_NUM_1;
constexpr gpio_num_t PIN_CS_DIFF_A = GPIO_NUM_15;

constexpr gpio_num_t PIN_DIFF_B_IN_1 = GPIO_NUM_6;
constexpr gpio_num_t PIN_DIFF_B_IN_2 = GPIO_NUM_7;
constexpr gpio_num_t PIN_CS_DIFF_B = GPIO_NUM_3;

constexpr gpio_num_t PIN_GRIP_IN_1 = GPIO_NUM_4;
constexpr gpio_num_t PIN_GRIP_IN_2 = GPIO_NUM_5;
constexpr gpio_num_t PIN_SWT_B_GRIP = GPIO_NUM_8;
constexpr gpio_num_t PIN_SWT_A_GRIP = GPIO_NUM_21;

constexpr gpio_num_t PIN_PB_DIFF_DOWN = GPIO_NUM_9;
constexpr gpio_num_t PIN_PB_DIFF_UP = GPIO_NUM_10;
constexpr gpio_num_t PIN_PB_DIFF_LEFT = GPIO_NUM_11;
constexpr gpio_num_t PIN_PB_DIFF_RIGHT = GPIO_NUM_12;
constexpr gpio_num_t PIN_PB_GRIP_REV = GPIO_NUM_13;
constexpr gpio_num_t PIN_PB_GRIP_FWD = GPIO_NUM_14;

constexpr gpio_num_t PIN_SPI_MISO = GPIO_NUM_16;
constexpr gpio_num_t PIN_SPI_MOSI = GPIO_NUM_17;
constexpr gpio_num_t PIN_SPI_SCK = GPIO_NUM_18;

constexpr gpio_num_t PIN_RXC = GPIO_NUM_47;
constexpr gpio_num_t PIN_TXC = GPIO_NUM_48;

#endif // __CONFIG_LOCAL_HPP__
