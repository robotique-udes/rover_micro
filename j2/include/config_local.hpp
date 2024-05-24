#ifndef __CONFIG_LOCAL_HPP__
#define __CONFIG_LOCAL_HPP__

// This file should be use to change parameter before uploading into a new
// device of the same type. For example, all propulsion motors should have the
// same code other then the constant specified here.
#include <cstdint>
#include "Arduino.h"
#include "rover_can_lib/constant.hpp"

constexpr uint16_t DEVICE_ID = (uint16_t)RoverCanLib::Constant::eDeviceId::REARRIGHT_MOTOR;

constexpr gpio_num_t PIN_J2_EN_1 = GPIO_NUM_8;
constexpr gpio_num_t PIN_J2_EN_2 = GPIO_NUM_16;

constexpr gpio_num_t PIN_J2_IN_1 = GPIO_NUM_21;
constexpr gpio_num_t PIN_J2_IN_2 = GPIO_NUM_38;

constexpr gpio_num_t PIN_PB_FWD = GPIO_NUM_5;
constexpr gpio_num_t PIN_PB_REV = GPIO_NUM_1;
constexpr gpio_num_t PIN_PB_CALIB = GPIO_NUM_4;

constexpr gpio_num_t PIN_LED_R = GPIO_NUM_3;
constexpr gpio_num_t PIN_LED_G = GPIO_NUM_9;
constexpr gpio_num_t PIN_LED_B = GPIO_NUM_10;

constexpr gpio_num_t PIN_SPI_SCK = GPIO_NUM_11;
constexpr gpio_num_t PIN_SPI_MOSI = GPIO_NUM_12;
constexpr gpio_num_t PIN_SPI_MISO = GPIO_NUM_13;
constexpr gpio_num_t PIN_SPI_CS_EN_SHAFT = GPIO_NUM_14;

#endif // __CONFIG_LOCAL_HPP__
