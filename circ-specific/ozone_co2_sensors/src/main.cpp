#include <Arduino.h>
#include <Wire.h>
#include "config.hpp"  // SDA=14, SCL=13, and ADDR=0x30

// ---------------- MQ137 easyC (0x30) minimal driver ----------------
namespace MQ137
{
    static constexpr uint8_t I2C_ADDR = 0x30;
    static constexpr uint8_t REG_VALUE0 = 0x00;  // 2-byte little-endian measurement
    static constexpr uint32_t I2C_HZ = 100000;   // 100 kHz is safest

    bool readRawLEu16(uint16_t& out)
    {
        Wire.beginTransmission(I2C_ADDR);
        Wire.write(REG_VALUE0);
        if (Wire.endTransmission(false) != 0)
            return false;  // repeated start
        if (Wire.requestFrom((int)I2C_ADDR, 2) != 2)
            return false;
        uint8_t lo = Wire.read();  // little-endian: low, then high
        uint8_t hi = Wire.read();
        out = (uint16_t(hi) << 8) | lo;  // combine as LE
        return true;
        // Note: your logs showed reg 0x01 mirrors 0x00; 0x00 is enough.
    }

    // Return ppb; on failure, returns -1
    int read_ppb()
    {
        uint16_t u16;
        if (!readRawLEu16(u16))
            return -1;
        // Based on your data, treat the value as ppb (500 -> 0.500 ppm).
        return (int)u16;  // 0..65535 ppb
    }
}  // namespace MQ137

// ---------------- app code ----------------
static int smooth_ppb(int ppb_new)
{
    // Simple IIR smoothing: y = 0.8*y + 0.2*x
    static bool init = false;
    static float y = 0.0f;
    if (ppb_new < 0)
        return -1;
    if (!init)
    {
        y = (float)ppb_new;
        init = true;
    }
    y = 0.8f * y + 0.2f * ppb_new;
    return (int)(y + 0.5f);
}

static void scanI2C()
{
    Serial.println("\n[I2C] scan:");
    for (uint8_t a = 1; a < 127; ++a)
    {
        Wire.beginTransmission(a);
        if (Wire.endTransmission() == 0)
            Serial.printf("  - 0x%02X\n", a);
        delay(2);
    }
}

void setup()
{
    Serial.begin(115200);
    delay(200);
    Wire.begin(MQ131_SDA, MQ131_SCL, MQ137::I2C_HZ);  // SDA=14, SCL=13
    Serial.printf("[I2C] SDA=%d SCL=%d @%lu Hz\n", MQ131_SDA, MQ131_SCL, (unsigned long)MQ137::I2C_HZ);
    scanI2C();
    Serial.println("MQ137 easyC @ 0x30 driver ready.");
}

void loop()
{
    int ppb = MQ137::read_ppb();
    if (ppb < 0)
    {
        Serial.println("Read NACK — check wiring/power.");
    }
    else
    {
        int sppb = smooth_ppb(ppb);
        float ppm = sppb / 1000.0f;  // interpret as ppb → ppm
        Serial.printf("NH3: %d ppb (smoothed: %d ppb = %.3f ppm)\n", ppb, sppb, ppm);
    }
}
