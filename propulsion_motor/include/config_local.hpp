#ifndef __CONFIG_LOCAL_HPP__
#define __CONFIG_LOCAL_HPP__

// This file should be use to change parameter before uploading into a new
// device of the same type. For example, all propulsion motors should have the
// same code other then the constant specified here.
#include <cstdint>
#include "rover_can_lib/constant.hpp"

constexpr uint16_t DEVICE_ID = (uint16_t)RoverCanLib::Constant::eDeviceId::REARRIGHT_MOTOR;

constexpr gpio_num_t PMW_MOT = GPIO_NUM_26;
constexpr gpio_num_t BTN_1 = GPIO_NUM_34;
constexpr gpio_num_t BTN_2 = GPIO_NUM_39;
constexpr gpio_num_t BTN_3 = GPIO_NUM_36;
constexpr gpio_num_t LED_1 = GPIO_NUM_32;
constexpr gpio_num_t LED_2 = GPIO_NUM_33;
constexpr gpio_num_t LED_3 = GPIO_NUM_25;
constexpr gpio_num_t CAN_RX = GPIO_NUM_17;
constexpr gpio_num_t CAN_TX = GPIO_NUM_16;

#endif // __CONFIG_LOCAL_HPP__
