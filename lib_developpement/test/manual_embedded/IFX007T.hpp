#include <Arduino.h>
#include <rover_lib2/LED/led_blinker.hpp>

#include <rover_can2/rover_can2.hpp>

#include <rover_lib2/actuators/PWM_generators/MCPWM.hpp>
#include <rover_lib2/actuators/motor_drivers/IFX007T.hpp>

constexpr gpio_num_t PIN_USER_LED = GPIO_NUM_6;

constexpr gpio_num_t PIN_PWM_A = GPIO_NUM_21;
constexpr gpio_num_t PIN_EN_A = GPIO_NUM_8;

constexpr gpio_num_t PIN_PWM_B = GPIO_NUM_38;
constexpr gpio_num_t PIN_EN_B = GPIO_NUM_16;

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

void setup(void)
{
    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif

    LED::LedBlinkerSoft led = LED::LedBlinkerSoft(IO::DigitalOutput(PIN_USER_LED), LED::BlinkPatterns::HEARTBEAT, 10);

    PWMGenerators::MCPWMTimer pwmTimer(1'000, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0);
    IO::DigitalOutput enableA(PIN_EN_A);
    PWMGenerators::MCPWM pwmA(PIN_PWM_A,
                              pwmTimer,
                              PWMGenerators::MCPWM::ePinOutputMode::ACTIVE_HIGH,
                              PWMGenerators::MCPWM::ePinPullMode::FLOATING);
    IO::DigitalOutput enableB(PIN_EN_B);
    PWMGenerators::MCPWM pwmB(PIN_PWM_B,
                              pwmTimer,
                              PWMGenerators::MCPWM::ePinOutputMode::ACTIVE_HIGH,
                              PWMGenerators::MCPWM::ePinPullMode::FLOATING);

    IFX007T motorDriver(enableA, pwmA, enableB, pwmB, false, MotorDriverT::eBrakeMode::BRAKE);
    motorDriver.init();
    motorDriver.setEnabled(true);
    // motorDriver.setCmd(25.0F);

    OneShotTimer<uint64_t, Time::millis> timerChange(2'000ULL);
    LOG_INFO(Logger::Nodes::Main, "Init done, starting loop!");
    for (EVER)
    {
        motorDriver.update();
        led.update();
        motorDriver.setCmd(50.0F);

        // if (timerChange.isReady())
        // {
        // }
    }
}

void loop() {}
