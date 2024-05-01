#ifndef __CONFIG_LOCAL_HPP__
#define __CONFIG_LOCAL_HPP__

// This file should be use to change parameter before uploading into a new
// device of the same type. For example, all propulsion motors should have the
// same code other then the constant specified here.
#include <cstdint>

#include "Arduino.h"
#include "rover_can_lib/constant.hpp"

constexpr uint16_t DEVICE_ID = (uint16_t)RoverCanLib::Constant::eDeviceId::PDB_CONTROLLER;

constexpr gpio_num_t CAM_ENABLE_A2 = GPIO_NUM_18;
constexpr gpio_num_t CAM_ENABLE_A2_ROT_SIDE = GPIO_NUM_21;
constexpr gpio_num_t CAM_ENABLE_A2_ROT_UP = GPIO_NUM_22;

constexpr gpio_num_t CAM_ENABLE_R1M_1 = GPIO_NUM_19;
constexpr gpio_num_t CAM_ENABLE_R1M_2 = GPIO_NUM_32;
constexpr gpio_num_t CAM_ENABLE_R1M_3 = GPIO_NUM_33;

constexpr gpio_num_t LIGHT_ENABLE = GPIO_NUM_23;
constexpr gpio_num_t KLAXON_ENABLE = GPIO_NUM_25;

constexpr gpio_num_t LED_R = GPIO_NUM_14;
constexpr gpio_num_t LED_B = GPIO_NUM_26;
constexpr gpio_num_t LED_G = GPIO_NUM_27;

constexpr gpio_num_t CAN_TX = GPIO_NUM_17;
constexpr gpio_num_t CAN_RX = GPIO_NUM_16;

#endif // __CONFIG_LOCAL_HPP__
