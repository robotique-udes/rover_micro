#include "Arduino.h"

#include "config_local.hpp"

#include "rover_helpers/helpers.hpp"
#include "actuators/IFX007T.hpp"
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

#warning TODO: Migrate
#warning TODO: Make parent object
class CUI_AMT222
{
public:
    static constexpr uint16_t INVALID_POSITION = 0xFFFF;
    static constexpr uint8_t REG_START = 0x00;
    static constexpr uint8_t REG_READ = 0x00;
    static constexpr uint8_t REG_RESET = 0x60;
    static constexpr uint8_t REG_SET_ZERO = 0x70;

    /// @brief Constructor
    /// @param spiBus_ Must call SPI*.begin() before passing to encoder
    /// @param pinCs_ Chip select low pin
    CUI_AMT222(SPIClass *spiBus_, gpio_num_t pinCs_)
    {
        ASSERT(spiBus_ == NULL);
        _pSpiBus = spiBus_;
        _pinCs = pinCs_;
    }

    void init()
    {
        pinMode(_pinCs, OUTPUT);
        digitalWrite(_pinCs, HIGH);

        _inited = true;
    }

    float getPosition(void)
    {
        return this->readPosition();
    }

    void setZero(void)
    {
        this->sendCmd(REG_SET_ZERO);
    }

    void reset(void)
    {
        this->sendCmd(REG_RESET);
    }

private:
    SPIClass *_pSpiBus = NULL;
    gpio_num_t _pinCs = GPIO_NUM_NC;
    bool _inited;

    bool isInited(void)
    {
        return _inited;
    }
    float readPosition()
    {
        ASSERT(!this->isInited());

        SPI.beginTransaction(SPISettings(2'000'000u, MSBFIRST, SPI_MODE0));
        digitalWrite(PIN_SPI_CS_EN_SHAFT, LOW);

        uint8_t positionByte0 = SPI.transfer(REG_START);
        uint8_t positionByte1 = SPI.transfer(REG_READ);

        uint16_t positionRaw = positionByte0 << 8 | positionByte1;
        positionRaw &= 0x3FFF;
        positionRaw = positionRaw >> 1;

        digitalWrite(PIN_SPI_CS_EN_SHAFT, HIGH);
        SPI.endTransaction();

        return MAP(positionRaw, 0, 2 << 12, 0.0f, 360.0f);
    }
    void sendCmd(uint8_t cmdRegister)
    {
        ASSERT(!this->isInited());

        SPI.beginTransaction(SPISettings(2'000'000u, MSBFIRST, SPI_MODE0));
        digitalWrite(PIN_SPI_CS_EN_SHAFT, LOW);

        SPI.transfer(REG_START);
        SPI.transfer(cmdRegister);

        digitalWrite(PIN_SPI_CS_EN_SHAFT, HIGH);
        SPI.endTransaction();
    }
};

void setup()
{
    Serial.begin(115200);
    delay(2000);

    LOG(WARN, "Init done starting!");

    IFX007T motorDriver;
    motorDriver.init(PIN_J2_EN_1, PIN_J2_EN_2, PIN_J2_IN_1, PIN_J2_IN_2, MotorDriver::eBrakeMode::COAST, false);
    motorDriver.attachRGBLed(PIN_LED_R, PIN_LED_G, PIN_LED_B);
    motorDriver.enable();
    motorDriver.setSpeed(0.0f);

    LimitSwitch switchFWD;
    switchFWD.init(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_FWD);
    LimitSwitch switchREV;
    switchREV.init(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_REV);

    LimitSwitch switchCalib;
    switchCalib.init(LimitSwitch::eLimitSwitchMode::PullUp, PIN_PB_CALIB);

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
            motorDriver.setSpeed(100.0f);
        }
        else if (switchREV.isClicked())
        {
            motorDriver.setSpeed(-100.0f);
        }
        else
        {
            motorDriver.setSpeed(0.0f);
        }
        motorDriver.update();

        if (switchCalib.isClicked())
        {
            shaftEncoder.setZero();
        }

        if (timerShaftEncoderRead.isDone())
        {
            LOG(INFO, "position: %f", shaftEncoder.getPosition());
        }
    }
}

void loop() {}
