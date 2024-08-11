#ifndef __CONFIG_LOCAL_HPP__
#define __CONFIG_LOCAL_HPP__

// This file should be use to change parameter before uploading into a new
// device of the same type. For example, all propulsion motors should have the
// same code other then the constant specified here.
#include <cstdint>
#include "Arduino.h"
#include "rover_can_lib/constant.hpp"

constexpr uint16_t DEVICE_ID = (uint16_t)RoverCanLib::Constant::eDeviceId::JL_CONTROLLER;

constexpr gpio_num_t PIN_JL_EN_1 = GPIO_NUM_14;
constexpr gpio_num_t PIN_JL_EN_2 = GPIO_NUM_13;

constexpr gpio_num_t PIN_JL_IN_1 = GPIO_NUM_21;
constexpr gpio_num_t PIN_JL_IN_2 = GPIO_NUM_12;

constexpr gpio_num_t PIN_PB_RIGHT = GPIO_NUM_1;
constexpr gpio_num_t PIN_PB_LEFT = GPIO_NUM_2;

constexpr gpio_num_t PIN_LS_RIGHT = GPIO_NUM_16;
constexpr gpio_num_t PIN_LS_LEFT = GPIO_NUM_17;

constexpr gpio_num_t PIN_SPI_SCK = GPIO_NUM_15;
constexpr gpio_num_t PIN_SPI_MOSI = GPIO_NUM_7;
constexpr gpio_num_t PIN_SPI_MISO = GPIO_NUM_6;
constexpr gpio_num_t PIN_SPI_CS_EN = GPIO_NUM_4;

constexpr gpio_num_t PIN_RXC = GPIO_NUM_47;
constexpr gpio_num_t PIN_TXC = GPIO_NUM_48;


#endif // __CONFIG_LOCAL_HPP__
