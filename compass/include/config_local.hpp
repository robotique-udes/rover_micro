#ifndef __CONFIG_LOCAL_HPP__
#define __CONFIG_LOCAL_HPP__

// This file should be use to change parameter before uploading into a new
// device of the same type. For example, all propulsion motors should have the
// same code other then the constant specified here.
#include <cstdint>
#include "Arduino.h"
#include "rover_can_lib/constant.hpp"

constexpr uint16_t DEVICE_ID = (uint16_t)RoverCanLib::Constant::eDeviceId::COMPASS;

constexpr gpio_num_t RX1_PIN = GPIO_NUM_12;
constexpr gpio_num_t TX1_PIN = GPIO_NUM_13;
constexpr gpio_num_t CAN_TX = GPIO_NUM_47;
constexpr gpio_num_t CAN_RX = GPIO_NUM_48;

#endif // __CONFIG_LOCAL_HPP__
