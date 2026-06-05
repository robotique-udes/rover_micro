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
#define PCB_ROVER_J345_REV1

// ===============================================================================================================================
// PCB Specific - Only change after pcb revisions
// ===============================================================================================================================
#if defined(PCB_ROVER_J345_REV1)

constexpr gpio_num_t PIN_USER_LED = GPIO_NUM_6;

// CAN
constexpr gpio_num_t PIN_CAN_LED = GPIO_NUM_9;
constexpr gpio_num_t PIN_CAN_RX = GPIO_NUM_5;
constexpr gpio_num_t PIN_CAN_TX = GPIO_NUM_4;

// SPI BUS
constexpr gpio_num_t PIN_SPI_MOSI = GPIO_NUM_48;
constexpr gpio_num_t PIN_SPI_MISO = GPIO_NUM_21;
constexpr gpio_num_t PIN_SPI_SCK = GPIO_NUM_47;

// I2C
constexpr gpio_num_t PIN_I2C_SDA = GPIO_NUM_11;
constexpr gpio_num_t PIN_I2C_SCL = GPIO_NUM_12;

// Motor
constexpr gpio_num_t PIN_J34_L_PWM = GPIO_NUM_15;
constexpr gpio_num_t PIN_J34_L_DIR = GPIO_NUM_16;
constexpr gpio_num_t PIN_J34_L_CS = GPIO_NUM_7;

constexpr gpio_num_t PIN_J34_R_PWM = GPIO_NUM_44;
constexpr gpio_num_t PIN_J34_R_DIR = GPIO_NUM_43;
constexpr gpio_num_t PIN_J34_R_CS = GPIO_NUM_14;

constexpr gpio_num_t PIN_J5_PWM = GPIO_NUM_17;
constexpr gpio_num_t PIN_J5_DIR = GPIO_NUM_13;

// Buttons
constexpr gpio_num_t PIN_PB_J34_CALIB = GPIO_NUM_40;
constexpr gpio_num_t PIN_PB_J3_FWD = GPIO_NUM_42;
constexpr gpio_num_t PIN_PB_J3_REV = GPIO_NUM_41;
constexpr gpio_num_t PIN_PB_J4_FWD = GPIO_NUM_2;
constexpr gpio_num_t PIN_PB_J4_REV = GPIO_NUM_38;
constexpr gpio_num_t PIN_PB_J5_OPEN = GPIO_NUM_1;
constexpr gpio_num_t PIN_PB_J5_CLOSE = GPIO_NUM_39;

#endif  // PCB_ROVER_J345_REV1

#endif  // CONFIG_HPP
