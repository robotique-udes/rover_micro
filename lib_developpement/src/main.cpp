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
 * [WARNING] For intellisense to work, replace the default .vscode inside the lib folder with the one generated under
 * lib_developpement
 *
 * Backup all your manuel tests under:
 * lib_developpement/test/manual_embedded
 */

#include <Arduino.h>
#include <rover_lib2/LED/led_blinker.hpp>



#include "rover_lib2/actuators/dc.hpp"
#include "rover_lib2/motor_drivers/IFX9201SG.hpp"
#include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"

#include "rover_lib2/sensors/encoder/AMT222X.hpp"
#include "rover_lib2/sensors/push_button.hpp"
#include "rover_lib2/helpers/loop_timer.hpp"

#include "rover_lib2/filters/low_pass_EMA.hpp"

constexpr gpio_num_t PIN_USER_LED = GPIO_NUM_6;
constexpr gpio_num_t PIN_J34_L_PWM = GPIO_NUM_16;
constexpr gpio_num_t PIN_J34_L_DIR = GPIO_NUM_15;
constexpr gpio_num_t PIN_J34_L_CS = GPIO_NUM_2;

constexpr gpio_num_t PIN_SPI_MOSI = GPIO_NUM_13;
constexpr gpio_num_t PIN_SPI_MISO = GPIO_NUM_12;
constexpr gpio_num_t PIN_SPI_SCK = GPIO_NUM_14;

constexpr gpio_num_t PIN_PB_CALIB = GPIO_NUM_1;
constexpr gpio_num_t PIN_FWD = GPIO_NUM_11;
constexpr gpio_num_t PIN_REV = GPIO_NUM_10;

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);
DEFINE_LOG_NODE(MainPlot, Logger::eNodeState::ON);

void setup(void)
{
    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif

    LED::LedBlinkerSoft led(IO::DigitalOutput(PIN_USER_LED), LED::BlinkPatterns::HEARTBEAT);
    led.init();

    LOG_INFO(Logger::Nodes::Main, "Init done, starting loop!");
    LoopTimer<uint64_t, &Time::millis> updateTimer(1);
    for (EVER)
    {

        led.update();
    }
}

void loop()
{
    // Don't use
}
