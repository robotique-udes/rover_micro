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
#include "rover_lib2/actuators/motor_drivers/IFX9201SG.hpp"
#include "rover_lib2/actuators/PWM_generators/MCPWM.hpp"

#include "rover_lib2/sensors/encoder/AMT222X.hpp"

constexpr gpio_num_t PIN_USER_LED = GPIO_NUM_6;
constexpr gpio_num_t PIN_J34_L_PWM = GPIO_NUM_16;
constexpr gpio_num_t PIN_J34_L_DIR = GPIO_NUM_15;
constexpr gpio_num_t PIN_J34_L_CS = GPIO_NUM_7;

constexpr gpio_num_t PIN_SPI_MOSI = GPIO_NUM_48;
constexpr gpio_num_t PIN_SPI_MISO = GPIO_NUM_21;
constexpr gpio_num_t PIN_SPI_SCK = GPIO_NUM_47;

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

void setup(void)
{
    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif

    LED::LedBlinkerSoft led = LED::LedBlinkerSoft(IO::DigitalOutput(PIN_USER_LED), LED::BlinkPatterns::HEARTBEAT);
    led.init();

    PWMGenerators::MCPWMTimer pwmTimer(1'000, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0);
    PWMGenerators::MCPWM pwm(PIN_J34_L_PWM,
                             pwmTimer,
                             PWMGenerators::MCPWM::ePinOutputMode::ACTIVE_HIGH,
                             PWMGenerators::MCPWM::ePinPullMode::PULL_DOWN);
    IFX9201SG<PWMGenerators::MCPWM> motorDriver(pwm, PIN_J34_L_DIR, false);

    SPIBus spiMotor(spi_host_device_t::SPI1_HOST, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCK, 32U);
    Encoders::AMT222X encoder = {spiMotor, PIN_J34_L_CS, "J34_L", false};

    Actuators::DC<IFX9201SG<PWMGenerators::MCPWM>> actuator(Actuators::eControlType::SPEED,
                                                            Actuators::eFeedbackType::OPEN_LOOP,
                                                            motorDriver);
    actuator.init();
    actuator.setSpeed(50.0F);

    LOG_INFO(Logger::Nodes::Main, "Init done, starting loop!");
    for (EVER)
    {
        actuator.update();
        led.update();
    }
}

void loop() {}
