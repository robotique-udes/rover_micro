#ifndef CONFIG_HPP
#define CONFIG_HPP

#include <cstdint>
#include <driver/gpio.h>

constexpr gpio_num_t LED_BLTN = GPIO_NUM_2;
constexpr gpio_num_t MQ8_AOUT = GPIO_NUM_12;
constexpr gpio_num_t MQ131_SCL = GPIO_NUM_13;
constexpr gpio_num_t MQ131_SDA = GPIO_NUM_14;

constexpr uint8_t ADDR = 0x30;

#endif