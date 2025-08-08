#include <Arduino.h>
#include <rover_lib2/LED/led_blinker.hpp>
#include <rover_lib2/sensors/encoder/AMT222A.hpp>
#include <rover_lib2/communication/SPI/SPI_bus.hpp>

#include <rover_lib2/actuators/motor_drivers/IFX9201SG.hpp>
#include <rover_lib2/actuators/PWM_generators/MCPWM.hpp>

#include <rover_lib2/sensors/push_button.hpp>

constexpr gpio_num_t PIN_USER_LED = GPIO_NUM_6;

constexpr gpio_num_t PIN_SPI_SCK = GPIO_NUM_14;
constexpr gpio_num_t PIN_SPI_MISO = GPIO_NUM_12;
constexpr gpio_num_t PIN_SPI_MOSI = GPIO_NUM_13;
constexpr gpio_num_t PIN_SPI_CS = GPIO_NUM_2;

constexpr gpio_num_t PIN_MOTOR_PWM = GPIO_NUM_43;
constexpr gpio_num_t PIN_MOTOR_DIR = GPIO_NUM_44;
constexpr gpio_num_t PIN_MOTOR_DIS = GPIO_NUM_NC;

constexpr gpio_num_t PIN_PB_FWD = GPIO_NUM_11;
constexpr gpio_num_t PIN_PB_REV = GPIO_NUM_10;
constexpr gpio_num_t PIN_PB_CALIB = GPIO_NUM_1;

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

void setup(void)
{
    Serial.begin(115200);
#if defined(DEBUG)
    delay(1000);
#endif

    SPIBus spi(spi_host_device_t::SPI2_HOST, PIN_SPI_MOSI, PIN_SPI_MISO, PIN_SPI_SCK, 32U);
    AMT222A encoder(spi, PIN_SPI_CS, true);
    encoder.init();

    PWMGenerators::MCPWMTimer _pwmTimer(15'000.0F, PWMGenerators::MCPWMTimer::eMCPWMGroupID::GROUP_0);
    PWMGenerators::MCPWM _pwmGen(PIN_MOTOR_PWM, _pwmTimer);
    IFX9201SG<PWMGenerators::MCPWM> motor(_pwmGen, PIN_MOTOR_DIR, false, PIN_MOTOR_DIS, MotorDriverT::eBrakeMode::BRAKE);
    motor.init();

    PushButton pbFwd(PIN_PB_FWD);
    PushButton pbRev(PIN_PB_REV);
    PushButton pbCalib(PIN_PB_CALIB);

    LED::LedBlinkerSoft led = LED::LedBlinkerSoft(IO::DigitalOutput(PIN_USER_LED), LED::BlinkPatterns::HEARTBEAT);
    led.init();

    LOG_INFO(Logger::Nodes::Main, "Init done, starting loop!");
    for (EVER)
    {
        motor.update();
        led.update();
        encoder.update();

        if (pbFwd.isClicked())
        {
            motor.setCmd(100.0F);
        }
        else if (pbRev.isClicked())
        {
            motor.setCmd(-100.0F);
        }
        else
        {
            motor.setCmd(0.0F);
        }

        if (pbCalib.isClicked())
        {
            encoder.calib(10.0F);
        }

        LOG_INFO(Logger::Nodes::Main, "encoder.getPosition(): %f", encoder.getPosition());
    }
}

void loop() {}
