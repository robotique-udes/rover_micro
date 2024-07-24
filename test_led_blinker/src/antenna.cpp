#include "Arduino.h"
#include "WiFi.h"
#include "WiFiUdp.h"
#include "rover_helpers/helpers.hpp"
#include "rover_helpers/chrono.hpp"

#define PUL 27
#define DIR 26
#define EN 25
#define MICRO_STEPS 800
#define RATIO_MOTOR 99.51

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

// UDP ans Wifi related
// const char *ssid = "roverAntenna";
// const char *password = "roverAntenna";
// IPAddress local_ip(192, 168, 144, 100);
// IPAddress gateway(192, 168, 144, 1);
// IPAddress subnet(255, 255, 255, 0);
// IPAddress host_ip(192, 168, 144, 50);

// WiFiUDP udp;
// unsigned int localUdpPort = 1234; // local port to listen on
// const char *replyPacket = "pong"; // a reply string to send back

// Timers
RoverHelpers::Timer<unsigned long, millis> timerSend(100);
RoverHelpers::Chrono<unsigned long, millis> chronoWatchdog;

// Global variables
bool motorEnable = false;
unsigned long periode;
uint32_t previousFreq = 0;
SemaphoreHandle_t sephamoreParam = NULL;

void setup()
{
    Serial.begin(115200);

    Serial.printf("started setup");

    pinMode(DIR, OUTPUT);
    pinMode(EN, OUTPUT);
    pinMode(PUL, OUTPUT);
    digitalWrite(EN, LOW);
    digitalWrite(DIR, LOW);

    // // Set up the ESP32 as an access point
    // Serial.println("Setting up AP (Access Point)...");
    // WiFi.softAP(ssid, password);
    // WiFi.softAPConfig(local_ip, gateway, subnet);

    // // Print the IP address of the access point
    // IPAddress IP = WiFi.softAPIP();
    // Serial.print("AP IP address: ");
    // Serial.println(IP);

    // // Start listening for UDP packets
    // udp.begin(localUdpPort);
    // Serial.printf("Now listening at IP %s, UDP port %d\n", IP.toString().c_str(), localUdpPort);
    // sephamoreParam = xSemaphoreCreateMutex();

    xTaskCreatePinnedToCore(
        motorLoop,   // Function to implement the task
        "motorLoop", // Name of the task
        4096,        // Stack size in words
        NULL,        // Task input parameter
        5,           // Priority of the task
        NULL,        // Task handle.
        0            // Core where the task should run
    );

    RoverHelpers::Timer<unsigned long, millis> timerUpdateParam(100);
    RoverHelpers::Chrono<unsigned long, millis> chronoWatchdogUpdate;
    RoverHelpers::Chrono<unsigned long, micros> chronoPulse;
    uint32_t newPeriode = 100;
    bool newEnable = true;
    bool pulse = false;

    Serial.printf("Setup finish");

    for (;;)
    {
        // if (timerUpdateParam.isDone())
        // {
        //     if (xSemaphoreTake(sephamoreParam, (TickType_t)0))
        //     {
        //         chronoWatchdogUpdate.restart();
        //         newPeriode = periode;
        //         newEnable = motorEnable;
        //         Serial.printf("Pram Updated\n");
        //     }
        // }

        // if (chronoWatchdogUpdate.getTime() > 500)
        // {
        //     newEnable = false;
        //     Serial.printf("Watchdog activated\n");
        //     chronoWatchdogUpdate.restart();
        // }
        // if (chronoPulse.getTime() >= newPeriode && newEnable)
        // {
            // pulse = pulse == HIGH ? LOW : HIGH;
            // digitalWrite(PUL, pulse);
            // // chronoPulse.restart();
            // delayMicroseconds(100);
        // }
        // recvAbtr();
    }
}

void loop() {}

void motorLoop(void *pvParameters)
{
    RoverHelpers::Timer<unsigned long, millis> timerUpdateParam(100);
    RoverHelpers::Chrono<unsigned long, millis> chronoWatchdogUpdate;
    RoverHelpers::Chrono<unsigned long, micros> chronoPulse;
    uint32_t newPeriode = 100;
    bool newEnable = true;
    bool pulse = false;

    for (;;)
    {
        // if (timerUpdateParam.isDone())
        // {
        //     if (xSemaphoreTake(sephamoreParam, (TickType_t)0))
        //     {
        //         chronoWatchdogUpdate.restart();
        //         newPeriode = periode;
        //         newEnable = motorEnable;
        //         Serial.printf("Pram Updated\n");
        //     }
        // }

        // if (chronoWatchdogUpdate.getTime() > 500)
        // {
        //     newEnable = false;
        //     Serial.printf("Watchdog activated\n");
        //     chronoWatchdogUpdate.restart();
        // }
        // if (chronoPulse.getTime() >= newPeriode && newEnable)
        // {
            // chronoPulse.restart();
            // pulse = !pulse;
            // digitalWrite(PUL, pulse);
            // xTaskDelayUntil(100);
            // Serial.printf("Step made\n");
        // }
    }
}

// void recvAbtr()
// {
// //     MsgAbtrCmd abtrData;
// //     uint8_t buffer[sizeof(MsgAbtrCmd)];
// //     int packetSize = udp.parsePacket();

// //     if (packetSize > 0)
// //     {
// //         udp.read(buffer, sizeof(buffer));
// //         abtrData.enable = ((MsgAbtrCmd *)buffer)->enable;
// //         abtrData.speed = ((MsgAbtrCmd *)buffer)->speed;
// //         Serial.printf("Received bytes: %f\n", abtrData.speed);

// //         // uint32_t freq = goal(abtrData);
// //         // motorSteps.setOn();
// //         // if (motorEnable && previousFreq != freq)
// //         // {
// //         //     // motorSteps.init(freq, 50.0f);
// //         //     // Serial.printf("Pulse\n");
// //         // }
// //         // previousFreq = freq;
// //         // chronoWatchdog.restart();
// //     }
// //     else if (chronoWatchdog.getTime() > 500ul)
// //     {
// //         motorEnable = false;
// //         // motorSteps.setOff();
// //         // motorSteps.init(1, 0.0f);
// //         previousFreq = 0;
// //         Serial.printf("No messages received\n");
// //     }

// //     MsgGPS testGPS = {1.2345, 6.7890};

// //     if (timerSend.isDone())
// //     {
// //         udp.beginPacket(host_ip, localUdpPort);
// //         udp.write((uint8_t *)&testGPS, sizeof(testGPS));
// //         udp.endPacket();
// //         // Serial.printf("Sent %f to %s:%d\n", testGPS.lattitude, udp.remoteIP().toString().c_str(), udp.remotePort());
// //     }
// }

// void goal(MsgAbtrCmd abtrData)
// {
//     bool motorEnableTemp;
//     uint32_t periodeTemp;

//     if (abtrData.speed == 0.0f)
//     {
//         motorEnableTemp = false;
//     }
//     else if (abtrData.speed != 0.0f)
//     {
//         periodeTemp = 2 * PI * 1e6 / (MICRO_STEPS * abs(abtrData.speed) * RATIO_MOTOR);

//         if (abtrData.speed < 0)
//         {
//             digitalWrite(DIR, LOW);
//         }
//         else
//         {
//             digitalWrite(DIR, HIGH);
//         }

//         motorEnableTemp = true;
//     }

//     if (xSemaphoreTake(sephamoreParam, (TickType_t)0))
//     {
//         motorEnable = motorEnableTemp;
//         periode = periodeTemp;
//     }

//     // Serial.printf("motorEnable : %i", motorEnable);
//     // uint32_t motorFreq = 1.0 / ((double)periode * 2 * 1e-6);
//     // Serial.printf("StepTimer : %d, Motor frequency : %d \n", periode, motorFreq);
//     // return motorFreq;
// }

