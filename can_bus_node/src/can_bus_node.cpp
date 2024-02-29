#if !defined(ESP32)
#error CPU not supported
#endif

#include <Arduino.h>

#include "CAN.h"
#include "helpers/helpers.hpp"

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;

    LOG(INFO, "CAN Sender");

    // start the CAN bus at 1000 kbps
    CAN.setPins(18, 4);
    ASSERT(!CAN.begin(500E3), "Starting CAN failed!");

    for (EVER)
    {
        // send packet: id is 11 bits, packet can contain up to 8 bytes of data
        LOG(INFO, "Sending packet");

        CAN.beginPacket(0x12);
        CAN.write('h');
        CAN.write('e');
        CAN.write('l');
        CAN.write('l');
        CAN.write('o');
        CAN.endPacket();

        LOG(INFO, "Done");

        delay(1000);

        // send extended packet: id is 29 bits, packet can contain up to 8 bytes of data
        // Serial.print("Sending extended packet ... ");

        // CAN.beginExtendedPacket(0xabcdef);
        // CAN.write('w');
        // CAN.write('o');
        // CAN.write('r');
        // CAN.write('l');
        // CAN.write('d');
        // CAN.endPacket();

        // Serial.println("done");

        // delay(1000);
    }
}

void loop() {}
