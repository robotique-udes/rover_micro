#include <Arduino.h>
#include "WiFi.h"
#include "WiFiUdp.h"
#include "rover_helpers/helpers.hpp"
#include "rover_helpers/chrono.hpp"

#define PUL 37
#define DIR 38
#define EN 39
#define MICRO_STEPS 800
#define RATIO_MOTOR 99.51

#define MICROSECONDS_PER_SECOND 1000000
#define TICKS_PER_SECOND configTICK_RATE_HZ

// Structure
struct MsgAbtrCmd
{
    float speed;
    bool enable;
};

struct MsgGPS
{
    float lattitude;
    float longitude;
};

// functions
void recvAbtr();
void goal(MsgAbtrCmd abtrData);
void motorLoop(void *pvParameters);

// Global variables
bool motorEnable = false;
unsigned long periode;
uint32_t previousFreq = 0;
SemaphoreHandle_t sephamoreParam = NULL;

// Timers
RoverHelpers::Timer<unsigned long, millis> timerSend(100);
RoverHelpers::Chrono<unsigned long, millis> chronoWatchdog;

// UDP
WiFiUDP udp;
unsigned int localUdpPort = 1234; // local port to listen on
const char *replyPacket = "pong"; // a reply string to send back

// Wifi
const char *ssid = "roverAntenna";
const char *password = "roverAntenna";
IPAddress local_ip(192, 168, 144, 100);
IPAddress gateway(192, 168, 144, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress host_ip(192, 168, 144, 50);

// Temp for testing
bool motorEnableTemp = true;
uint32_t periodeTemp = 1;

void setup()
{
    Serial.begin(115200);

    pinMode(PUL, OUTPUT);
    pinMode(DIR, OUTPUT);
    pinMode(EN, OUTPUT);

    digitalWrite(DIR, LOW);
    digitalWrite(EN, HIGH);

    xTaskCreatePinnedToCore(
        motorLoop,   // Function to implement the task
        "motorLoop", // Name of the task
        8192,        // Stack size in words
        NULL,        // Task input parameter
        5,           // Priority of the task
        NULL,        // Task handle.
        0            // Core where the task should run
    );

    sephamoreParam = xSemaphoreCreateMutex();
}

void loop()
{
    recvAbtr();
    taskYIELD();
}

void motorLoop(void *pvParameters)
{
    RoverHelpers::Timer<unsigned long, millis> timerUpdateParam(100);
    RoverHelpers::Chrono<unsigned long, millis> chronoWatchdogUpdate;
    TickType_t xLastWakeTime;
    TickType_t xPeriode = 1;
    bool newEnable = true;
    bool pulse = false;

    while (true)
    {
        xLastWakeTime = xTaskGetTickCount();
        if (timerUpdateParam.isDone())
        {
            if (xSemaphoreTake(sephamoreParam, (TickType_t)0))
            {
                chronoWatchdogUpdate.restart();
                xPeriode = periode;
                newEnable = motorEnable;
                if (xSemaphoreGive(sephamoreParam) != pdTRUE)
                {
                    Serial.println("Failed to release semaphore");
                    // Handle semaphore give failure if needed
                }
            }
        }

        if (chronoWatchdogUpdate.getTime() > 500)
        {
            newEnable = false;
            Serial.printf("Watchdog activated\n");
            chronoWatchdogUpdate.restart();
        }
        else
        {
            // xLastWakeTime = xTaskGetTickCount();
            // const TickType_t xPeriode = (xPeriode * (MICROSECONDS_PER_SECOND / TICKS_PER_SECOND));
            xPeriode = periode;
            newEnable = motorEnable;
            pulse = pulse == HIGH ? LOW : HIGH;
            digitalWrite(PUL, pulse);
            chronoWatchdogUpdate.restart();
            xTaskDelayUntil(&xLastWakeTime, xPeriode);
        }
    }
}

void recvAbtr()
{
    MsgAbtrCmd abtrData;
    uint8_t buffer[sizeof(MsgAbtrCmd)];
    int packetSize = udp.parsePacket();

    if (packetSize > 0)
    {
        udp.read(buffer, sizeof(buffer));
        abtrData.enable = ((MsgAbtrCmd *)buffer)->enable;
        abtrData.speed = ((MsgAbtrCmd *)buffer)->speed;
        Serial.printf("Received bytes: %f\n", abtrData.speed);

        goal(abtrData);
        chronoWatchdog.restart();
    }
    else if (chronoWatchdog.getTime() > 500ul)
    {
        motorEnable = false;
        // Serial.printf("No messages received\n");
    }

    // Temp for testing
    if (xSemaphoreTake(sephamoreParam, (TickType_t)0))
    {
        motorEnable = motorEnableTemp;
        periode = periodeTemp;
        xSemaphoreGive(sephamoreParam);
    }
}

void goal(MsgAbtrCmd abtrData)
{
    bool motorEnableTemp;
    uint32_t periodeTemp;

    if (abtrData.speed == 0.0f)
    {
        motorEnableTemp = false;
    }
    else if (abtrData.speed != 0.0f)
    {
        periodeTemp = 2 * PI * 1e6 / (MICRO_STEPS * abs(abtrData.speed) * RATIO_MOTOR);

        if (abtrData.speed < 0)
        {
            digitalWrite(DIR, LOW);
        }
        else
        {
            digitalWrite(DIR, HIGH);
        }

        motorEnableTemp = true;
    }

    if (xSemaphoreTake(sephamoreParam, (TickType_t)0))
    {
        motorEnable = motorEnableTemp;
        periode = periodeTemp;
        xSemaphoreGive(sephamoreParam);
    }
}
