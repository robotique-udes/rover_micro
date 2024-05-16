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

#endif // __CONFIG_LOCAL_HPP__
