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
#define PCB_ROVER_JL_REV0

constexpr float ALIM_VOLTAGE = 25.2F;
constexpr float MAX_MOTOR_VOLTAGE = 12.0F;

// ===============================================================================================================================
// PCB Specific - Only change after pcb revisions
// ===============================================================================================================================
#if defined(PCB_ROVER_JL_REV0)

constexpr gpio_num_t PIN_MOTOR_A_IN = GPIO_NUM_1;
constexpr gpio_num_t PIN_MOTOR_A_EN = GPIO_NUM_2;

constexpr gpio_num_t PIN_MOTOR_B_IN = GPIO_NUM_48;
constexpr gpio_num_t PIN_MOTOR_B_EN = GPIO_NUM_47;

constexpr gpio_num_t PIN_ENC_MISO = GPIO_NUM_4;
constexpr gpio_num_t PIN_ENC_MOSI = GPIO_NUM_5;
constexpr gpio_num_t PIN_ENC_CLK = GPIO_NUM_6;
constexpr gpio_num_t PIN_ENC_CS = GPIO_NUM_7;

constexpr gpio_num_t PIN_CAN_LED = GPIO_NUM_21;
constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_9;
constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_10;

constexpr gpio_num_t PIN_PB_CALIB = GPIO_NUM_13;
constexpr gpio_num_t PIN_PB_FWD = GPIO_NUM_14;
constexpr gpio_num_t PIN_PB_REV = GPIO_NUM_38;

constexpr gpio_num_t PIN_POT_ANAL = GPIO_NUM_15;

#endif  // PCB_ROVER_JL_REV0

#endif  // CONFIG_HPP
