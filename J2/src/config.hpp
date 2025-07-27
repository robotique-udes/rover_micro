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
#define PCB_ROVER_J2_REV0

// ===============================================================================================================================
// PCB Specific - Only change after pcb revisions
// ===============================================================================================================================
#if defined(PCB_ROVER_J2_REV0)

constexpr gpio_num_t PIN_ENC_MISO = GPIO_NUM_12;
constexpr gpio_num_t PIN_ENC_MOSI = GPIO_NUM_13;
constexpr gpio_num_t PIN_ENC_CLK = GPIO_NUM_17;
constexpr gpio_num_t PIN_ENC_CS = GPIO_NUM_2;

constexpr gpio_num_t PIN_UART_TX = GPIO_NUM_6;
constexpr gpio_num_t PIN_UART_RX = GPIO_NUM_7;
constexpr int UART_BAUD_RATE = 921600;

constexpr gpio_num_t PIN_CAN_LED = GPIO_NUM_21;
constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_9;
constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_10;

constexpr gpio_num_t PIN_PB_CALIB = GPIO_NUM_13;
constexpr gpio_num_t PIN_PB_PLUS = GPIO_NUM_11;
constexpr gpio_num_t PIN_PB_NEG = GPIO_NUM_10;

constexpr gpio_num_t PIN_POT_ANAL = GPIO_NUM_1;

#endif  // PCB_ROVER_J1_REV0

#endif  // CONFIG_HPP
