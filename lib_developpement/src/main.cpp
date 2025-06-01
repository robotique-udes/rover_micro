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
#include <rover_lib2/LED/led_blinker.hpp>

#include <rover_can2/rover_can2.hpp>

#include <rover_lib2/actuators/PWM_generators/MCPWM.hpp>
#include <rover_lib2/actuators/motor_drivers/IFX9201SG.hpp>

constexpr gpio_num_t PIN_USER_LED = GPIO_NUM_6;
constexpr gpio_num_t PIN_CAN_LED = GPIO_NUM_9;

constexpr gpio_num_t PIN_DIR = GPIO_NUM_44;
constexpr gpio_num_t PIN_PWM = GPIO_NUM_43;
constexpr gpio_num_t PIN_N_EN = GPIO_NUM_NC;

constexpr float PWM_FREQUENCY = 15'000.0F;

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

void setup(void)
{
    // Fix for PWM and DIR pins on gpio with default behaviors
    {
        IO::DigitalOutput io(PIN_PWM, IO::eIOState::LOW_);
        io.write(IO::eIOState::LOW_);
    }

    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif

    LED::LedBlinkerSoft led = LED::LedBlinkerSoft(IO::DigitalOutput(PIN_USER_LED), LED::BlinkPatterns::HEARTBEAT);
    led.init();

    PWMGenerators::MCPWMTimer pwmTimer(PWM_FREQUENCY, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0);
    PWMGenerators::MCPWM pwmGen(PIN_PWM,
                                pwmTimer,
                                PWMGenerators::MCPWM::ePinOutputMode::ACTIVE_HIGH,
                                PWMGenerators::MCPWM::ePinPullMode::FLOATING);

    IFX9201SG motorDriver(pwmGen, PIN_DIR, PIN_N_EN);
    motorDriver.init();
    motorDriver.setEnabled(true);
    motorDriver.setCmd(25.0F);

    LoopTimer<uint64_t, Time::millis> timerCmdIncrease(100ULL);
    float cmd = -100.0F;
    LOG_INFO(Logger::Nodes::Main, "Init done, starting loop!");
    for (EVER)
    {
        motorDriver.update();
        led.update();

        if (timerCmdIncrease.isReady())
        {
            motorDriver.setCmd((cmd += 1.0F));
        }
    }
}

void loop() {}
