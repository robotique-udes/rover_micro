/**
 * @file device_config.hpp
 *
 * @brief This file should be use to change parameter before uploading into a new device of the same type.
 */

#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <cstdint>
#include <driver/gpio.h>

// ===============================================================================================================================
// Device specific
// ===============================================================================================================================
#define PCB_ROVER_SCIENCE

// ===============================================================================================================================
// PCB Specific - Only change after pcb revisions
// ===============================================================================================================================
#if defined(PCB_ROVER_SCIENCE)

constexpr float ALIM_VOLTAGE = 25.2F;
constexpr float MAX_MOTOR_VOLTAGE = 18.0F;

constexpr gpio_num_t PIN_LIN_ACT_1 = GPIO_NUM_18;
constexpr gpio_num_t PIN_LIN_ACT_2 = GPIO_NUM_8;

constexpr gpio_num_t PIN_CAN_LED = GPIO_NUM_5;
constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_47;
constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_48;

constexpr gpio_num_t PIN_LIN_ACT_LS = GPIO_NUM_21;

#endif  // PCB_ROVER_J1_REV0

#endif  // CONFIG_HPP
