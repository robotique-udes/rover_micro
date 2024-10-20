#include <Arduino.h>
#include "WiFi.h"
#include "WiFiUdp.h"
#include "rover_helpers/helpers.hpp"
#include "rover_helpers/chrono.hpp"

#define PUL 37
#define DIR 38
#define EN 39
#define MICRO_STEPS 1600
#define RATIO_MOTOR 99.51

#define UDP_PORT 1234

// Structure
struct sMsgAbtrCmd
{
    float speed;
    bool enable;
};

struct sMsgGPS
{
    float latitude;
    float longitude;
};

// functions
void recvAbtr();
void goal(sMsgAbtrCmd abtrData);
void motorLoop(void *pvParameters);

// Global variables
bool motorEnable = false;
unsigned long periode;
uint32_t previousFreq = 0;
SemaphoreHandle_t sephamoreParam = NULL;

void setup()
{
    Serial.begin(115200);

    pinMode(PUL, OUTPUT);
    pinMode(DIR, OUTPUT);
    pinMode(EN, OUTPUT);

    digitalWrite(DIR, LOW);
    digitalWrite(EN, LOW);

    while (sephamoreParam == NULL)
        sephamoreParam = xSemaphoreCreateMutex();

    // Wifi
    const char *ssid = "roverAntenna";
    const char *password = "roverAntenna";
    IPAddress local_ip(192, 168, 140, 60);
    IPAddress gateway(192, 168, 140, 1);
    IPAddress subnet(255, 255, 255, 0);

    // Set up the ESP32 as an access point
    LOG(INFO, "Setting up AP (Access Point)...");
    WiFi.softAP(ssid, password);
    WiFi.softAPConfig(local_ip, gateway, subnet);

    // Print the IP address of the access point
    IPAddress IP = WiFi.softAPIP();
    LOG(INFO,"AP IP address: %s", IP.toString().c_str());

    xTaskCreatePinnedToCore(
        motorLoop,
        "motorLoop",
        4096,
        NULL,
        1,
        NULL,
        1);

    for (;;)
    {
        recvAbtr();
    }
}

void loop() {}

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
                    LOG(WARN, "Failed to release semaphore");
                }
            }
        }

        if (chronoWatchdogUpdate.getTime() > 500)
        {
            newEnable = false;
            LOG(INFO, "Watchdog activated");
            chronoWatchdogUpdate.restart();
        }
        else if (newEnable)
        {
            pulse = !pulse;
            digitalWrite(PUL, pulse);
            chronoWatchdogUpdate.restart();
            delayMicroseconds(periode);
            LOG(INFO, "pulse");
        }
    }
}

void recvAbtr()
{
    // UDP
    WiFiUDP udp;
    unsigned int localUdpPort = UDP_PORT;

    // Start listening for UDP packets
    udp.begin(localUdpPort);
    IPAddress IP = WiFi.softAPIP();
    IPAddress host_ip(192, 168, 140, 100);
    LOG(INFO, "Now listening at IP %s, UDP port %d\n", IP.toString().c_str(), localUdpPort);

    RoverHelpers::Timer<unsigned long, millis> timerSend(100);
    RoverHelpers::Chrono<unsigned long, millis> chronoWatchdog;

    while (true)
    {
        sMsgAbtrCmd abtrData;
        uint8_t buffer[sizeof(sMsgAbtrCmd)];
        int packetSize = udp.parsePacket();

        // Receive message
        if (packetSize > 0)
        {
            udp.read(buffer, sizeof(buffer));
            abtrData.enable = ((sMsgAbtrCmd *)buffer)->enable;
            abtrData.speed = ((sMsgAbtrCmd *)buffer)->speed;
            goal(abtrData);
            chronoWatchdog.restart();
            LOG(INFO, "Message received : %f", abtrData.speed)
        }
        else if (chronoWatchdog.getTime() > 500ul)
        {
            motorEnable = false;
        }

        sMsgGPS testGPS = {1.2345, 6.7890};

        // // Send message
        // if (timerSend.isDone())
        // {
        //     udp.beginPacket(host_ip, localUdpPort);
        //     udp.write((uint8_t *)&testGPS, sizeof(testGPS));
        //     udp.endPacket();
        // }
    }
}

void goal(sMsgAbtrCmd abtrData)
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
        if (xSemaphoreGive(sephamoreParam) != pdTRUE)
        {
            LOG(WARN, "Failed to release semaphore");
        }
    }
}
