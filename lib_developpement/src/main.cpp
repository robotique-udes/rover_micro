/**
 * @file main.hpp
 * @brief Main for quick testing or library implementation.
 *
 * The following folder structure in vscode is recommended to make intellisense work in the lib folder
 * └── lib_developpement
 *   ├── .clang-format
 *   ├── .gitignore
 *   ├── include
 *   ├── .pio
 *   ├── platformio.ini
 *   ├── src
 *   ├── test
 *   └── .vscode
 * └── lib
 *   ├── .clang-format
 *   ├── lib_rover
 *   ├── rover_can2
 *   ├── rover_can_lib
 *   ├── rover_lib2
 *   └── .vscode
 *
 *  [WARNING] For intellisense to work, replace the default .vscode inside the lib folder with the one generated under
 * lib_developpement
 *
 * Backup all your manuel tests under:
 *  lib_developpement/test/manual_embedded
 */

#include <Arduino.h>
#include "rover_lib2/LED/led_blinker.hpp"

constexpr gpio_num_t PIN_USER_LED = GPIO_NUM_6;

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

void setup(void)
{
    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif

    LED::LedBlinkerSoft led = LED::LedBlinkerSoft(IO::DigitalOutput(PIN_USER_LED), LED::BlinkPatterns::HEARTBEAT, 10);

    LOG_INFO(Logger::Nodes::Main, "Init done, starting loop!");
    for (EVER)
    {
        led.update();
    }
}

void loop() {}
