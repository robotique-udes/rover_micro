#include "Arduino.h"
#include "helpers/timer.h"
#include "helpers/moving_average.h"
#include "helpers/log.h"

#define FREQ 200
#define PIN_PWM 4u

void setup()
{
    Serial.begin(115200);

    pinMode(PIN_PWM, OUTPUT);

    MovingAverage<uint16_t, 100u> avgPWM(1500u);
    Timer<unsigned long, micros> timerPulse(5000u);
    
    Serial.printf("Init done, starting Loop!\n");
    for (;;)
    {
        if (timerPulse.isDone())
        {
            digitalWrite(PIN_PWM, HIGH);
            switch (Serial.read())
            {
            case '1':
                avgPWM.addValue(1000);
                break;

            case '2':
                avgPWM.addValue(1500);
                break;

            case '3':
                avgPWM.addValue(2000);
                break;
            }
            
            delayMicroseconds(static_cast<uint16_t>(round(avgPWM.getAverage())));
            digitalWrite(PIN_PWM, LOW);
        }
    }
}
