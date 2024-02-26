#if !defined (ESP32)
#error CPU not supported
#endif

#include <Arduino.h>
#include "helpers/helpers.hpp"

void setup()
{
    Serial.begin(115200);

    for (EVER)
    {
        LOG(INFO, "Logging\n");
    }
}

void loop() {}
