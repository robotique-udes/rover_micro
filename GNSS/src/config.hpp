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
#define PCB_ROVER_GNSS_REV1

// ===============================================================================================================================
// PCB Specific - Only change after pcb revisions
// ===============================================================================================================================
#if defined(PCB_ROVER_GNSS_REV1)

// CAN
constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_5;
constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_4;
constexpr gpio_num_t PIN_LED_CAN = GPIO_NUM_2;

// UART GNSS
constexpr gpio_num_t PIN_UART_TX = GPIO_NUM_47;
constexpr gpio_num_t PIN_UART_RX = GPIO_NUM_13;

#endif  // PCB_ROVER_GNSS_REV1

#endif  // CONFIG_HPP