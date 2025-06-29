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
constexpr gpio_num_t PIN_J34_L_CS = GPIO_NUM_7;

constexpr gpio_num_t PIN_SPI_MOSI = GPIO_NUM_48;
constexpr gpio_num_t PIN_SPI_MISO = GPIO_NUM_21;
constexpr gpio_num_t PIN_SPI_SCK = GPIO_NUM_47;

constexpr gpio_num_t PIN_PB_CALIB = GPIO_NUM_40;
constexpr gpio_num_t PIN_FWD = GPIO_NUM_42;
constexpr gpio_num_t PIN_REV = GPIO_NUM_41;

DEFINE_LOG_NODE(Main, Logger::eNodeState::OFF);
DEFINE_LOG_NODE(MainPlot, Logger::eNodeState::ON);

void setup(void)
{
    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif

    LED::LedBlinkerSoft led(IO::DigitalOutput(PIN_USER_LED), LED::BlinkPatterns::HEARTBEAT);
    led.init();

    PWMGenerators::MCPWMTimer pwmTimer(1'000, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0);
    PWMGenerators::MCPWM pwm(PIN_J34_L_PWM,
                             pwmTimer,
                             PWMGenerators::MCPWM::ePinOutputMode::ACTIVE_HIGH,
                             PWMGenerators::MCPWM::ePinPullMode::PULL_DOWN);
    MotorDrivers::IFX9201SG<PWMGenerators::MCPWM> motorDriver(pwm, PIN_J34_L_DIR, false);

    SPIBus spiMotor(spi_host_device_t::SPI2_HOST, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCK, 32U);
    Filters::LowPassEMA lowPassPos(0.05F);
    Filters::LowPassEMA lowPassSpeed(0.20F);
    Encoders::AMT222X encoder = {spiMotor, PIN_J34_L_CS, "J34_L", lowPassPos, lowPassSpeed, false};

    Controllers::PID pidSpeed(50.0F, 12.5F, 0.1F, 100.0F, 5'000ULL);

    Actuators::DC<MotorDrivers::IFX9201SG<PWMGenerators::MCPWM>,
                  Encoders::AMT222X<Filters::LowPassEMA, Filters::LowPassEMA>,
                  Controllers::None,
                  Controllers::PID>
        actuator(Actuators::eControlType::SPEED, Actuators::eFeedbackType::OPEN_LOOP, motorDriver, &encoder, nullptr, &pidSpeed);
    actuator.init();

    PushButton calibPb(PIN_PB_CALIB);
    PushButton fwdPb(PIN_FWD);
    PushButton revPb(PIN_REV);

    LOG_INFO(Logger::Nodes::Main, "Init done, starting loop!");
    LoopTimer<uint64_t, &Time::millis> updateTimer(1);
    for (EVER)
    {
        led.update();

        if (!updateTimer.isReady())
        {
            continue;
        }

        actuator.update();

        LOG_INFO(Logger::Nodes::Main,
                 "actuator.getPosition(): %f | actuator.getSpeed(): %f",
                 actuator.getPosition(),
                 actuator.getSpeed());

        float targetSpeed;
        if (calibPb.isClicked())
        {
            actuator.calib(0.0F);
        }

        if (fwdPb.isClicked())
        {
            targetSpeed = 50.0F;
        }
        else if (revPb.isClicked())
        {
            targetSpeed = -100.0F;
        }
        else
        {
            targetSpeed = 0.0F;
        }

        actuator.setSpeed(targetSpeed);
        LOG_PLOT(Logger::Nodes::MainPlot, actuator.getSpeed());
        LOG_PLOT(Logger::Nodes::MainPlot, targetSpeed);
        LOG_PLOT(Logger::Nodes::MainPlot, actuator.getPosition());
    }
}

void loop()
{
    // Don't use
}
