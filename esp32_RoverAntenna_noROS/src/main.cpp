#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "rover_helpers/timer.hpp"
#include <semaphore>

#define PUL 27
#define DIR 26
#define EN 25

#define MICRO_STEPS 800
#define RATIO_MOTOR 99.51

#define RX1PIN 16
#define TX1PIN 17
#define BUFFER_SIZE 255

#define NB_ELEMENTS 20
#define SIZE_ELEMENTS 255

void goal();
void motorLoop(void *pvParameters);
void recvAbtr();

bool motorEnable = false;
bool stepper_direction;
uint8_t motorStep = HIGH;

// Creating mutex
SemaphoreHandle_t enableSemaphore = NULL;
SemaphoreHandle_t timerSemaphore = NULL;

// Set up the Wi-Fi network credentials for the ESP32 AP
const char *ssid = "roverAntenna";
const char *password = "roverAntenna";
IPAddress local_ip(192, 168, 144, 100);
IPAddress gateway(192, 168, 144, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress host_ip(192, 168, 144, 50);

WiFiUDP udp;
unsigned int localUdpPort = 1234; // local port to listen on
const char *replyPacket = "pong"; // a reply string to send back

RoverHelpers::Timer<unsigned long, millis> timerSend(100);
RoverHelpers::Timer<unsigned long, micros> timerStepper(200ul);

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

MsgAbtrCmd abtrData;

float led_blink = false;

TickType_t lastTimeStep;
TickType_t motorFreq;

void setup()
{
    Serial.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);
    pinMode(PUL, OUTPUT);
    pinMode(EN, OUTPUT);
    pinMode(DIR, OUTPUT);

    // Serial1.begin(9600, SERIAL_8N1, RX1PIN, TX1PIN);

    enableSemaphore = xSemaphoreCreateMutex();
    timerSemaphore = xSemaphoreCreateMutex();

    digitalWrite(EN, LOW);
    digitalWrite(DIR, HIGH);

    // Set up the ESP32 as an access point
    Serial.println("Setting up AP (Access Point)...");
    WiFi.softAP(ssid, password);
    WiFi.softAPConfig(local_ip, gateway, subnet);

    // Print the IP address of the access point
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);

    // Start listening for UDP packets
    udp.begin(localUdpPort);
    Serial.printf("Now listening at IP %s, UDP port %d\n", IP.toString().c_str(), localUdpPort);

    xTaskCreatePinnedToCore(
        motorLoop,   // Function to implement the task
        "motorLoop", // Name of the task
        2048,        // Stack size in words
        NULL,        // Task input parameter
        2,           // Priority of the task
        NULL,        // Task handle.
        0            // Core where the task should run
    );
}

void loop()
{
    goal();
    recvAbtr();
}

void motorLoop(void *pvParameters)
{
    for (;;)
    {
        Serial.printf("Before mutex\n");

        // if (xSemaphoreTake(xSemaphore, (TickType_t)0))
        // {
        //     motorStep = !motorStep;
        //     digitalWrite(PUL, motorStep);
        //     led_blink = !led_blink;
        //     digitalWrite(LED_BUILTIN, led_blink);
        //     // xTaskDelayUntil(&lastTimeStep, pdMS_TO_TICKS(10));
        //     // }
        //     xSemaphoreGive(xSemaphore);
        // }
    }
}

void recvAbtr()
{
    MsgAbtrCmd incomingPacket2;
    uint8_t buffer[sizeof(MsgAbtrCmd)];
    int packetSize = udp.parsePacket();

    if (packetSize > 0)
    {
        udp.read(buffer, sizeof(buffer));
        abtrData.enable = ((MsgAbtrCmd *)buffer)->enable;
        abtrData.speed = ((MsgAbtrCmd *)buffer)->speed;
        Serial.printf("Received bytes: %f\n", abtrData.speed);
    }

    MsgGPS testGPS = {1.2345, 6.7890};

    if (timerSend.isDone())
    {
        udp.beginPacket(host_ip, localUdpPort);
        udp.write((uint8_t *)&testGPS, sizeof(testGPS));
        udp.endPacket();
        // Serial.printf("Sent %f to %s:%d\n", testGPS.lattitude, udp.remoteIP().toString().c_str(), udp.remotePort());
    }
}

void goal()
{
    unsigned long stepTimer;

    if (abtrData.speed == 0.0f)
    {
        motorEnable = false;
    }
    else if (abtrData.speed != 0.0f)
    {
        stepTimer = 2 * PI * 1e6 / (MICRO_STEPS * abs(abtrData.speed) * RATIO_MOTOR);

        if (abtrData.speed < 0)
        {
            digitalWrite(DIR, LOW);
        }
        else
        {
            digitalWrite(DIR, HIGH);
        }

        if (xSemaphoreTake(enableSemaphore, (TickType_t)0))
        {
            motorEnable = true;
            xSemaphoreGive(enableSemaphore);
        }
        motorFreq = stepTimer;
        digitalWrite(EN, LOW);
    }
}