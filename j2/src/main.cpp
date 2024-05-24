#include "Arduino.h"

#include "config_local.hpp"

#include "rover_helpers/helpers.hpp"
#include "actuators/motor_drivers/IFX007T.hpp"
#include "sensors/encoders/CUI_AMT222.hpp"
#include "sensors/limit_switch.hpp"
#include "SPI.h"

// =============================================================================
// Temp
#warning TODO: Migrate
#define PIN_SPI_SCK GPIO_NUM_11
#define PIN_SPI_MOSI GPIO_NUM_12
#define PIN_SPI_MISO GPIO_NUM_13
#define PIN_SPI_CS_EN_SHAFT GPIO_NUM_14

// =============================================================================

void setup()
{
    Serial.begin(115200);
    delay(2000);

    LOG(WARN, "Init done starting!");

    IFX007T motorDriver(PIN_J2_EN_1, PIN_J2_EN_2, PIN_J2_IN_1, PIN_J2_IN_2, MotorDriver::eBrakeMode::BRAKE, false);
    motorDriver.init();
    motorDriver.attachRGBLed(PIN_LED_R, PIN_LED_G, PIN_LED_B);
    motorDriver.enable();
    motorDriver.setCmd(0.0f);

    LimitSwitch switchFWD(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_FWD);
    switchFWD.init();
    LimitSwitch switchREV(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_REV);
    switchREV.init();

    LimitSwitch switchCalib(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_CALIB);
    switchCalib.init();

    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, GPIO_NUM_NC);
    CUI_AMT222 shaftEncoder(&SPI, PIN_SPI_CS_EN_SHAFT);
    shaftEncoder.init();

    RoverHelpers::Timer<unsigned long, millis> timerShaftEncoderRead(20);
    RoverHelpers::Timer<unsigned long, millis> timer(500);
    int16_t i = 0;
    for (;;)
    {
        if (switchFWD.isClicked())
        {
            motorDriver.setCmd(100.0f);
        }
        else if (switchREV.isClicked())
        {
            motorDriver.setCmd(-100.0f);
        }
        else
        {
            motorDriver.setCmd(0.0f);
        }
        motorDriver.update();

        if (switchCalib.isClicked())
        {
            shaftEncoder.calib();
        }

        if (timerShaftEncoderRead.isDone())
        {
            LOG(INFO, "position: %f", shaftEncoder.getPosition());
        }
    }
}

void loop() {}
