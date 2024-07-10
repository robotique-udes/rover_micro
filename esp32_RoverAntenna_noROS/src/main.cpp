#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include "rover_helpers/timer.hpp"

// Set up the Wi-Fi network credentials for the ESP32 AP
const char *ssid = "roverAntenna";
const char *password = "roverAntenna";
IPAddress local_ip(192, 168, 144, 100);
IPAddress gateway(192, 168, 144, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress host_ip(192, 168, 144, 50);

WiFiUDP udp;
unsigned int localUdpPort = 1234; // local port to listen on
char incomingPacket[255];         // buffer for incoming packets
const char *replyPacket = "pong"; // a reply string to send back

RoverHelpers::Timer<unsigned long, millis> timerSend(100);

void recvAbtr();

float led_blink = false;

void setup()
{
    Serial.begin(115200);

    pinMode(LED_BUILTIN, OUTPUT);

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
}

void loop()
{

    recvAbtr();

    // Send a reply to the IP address and port that sent the incoming packet
    // udp.beginPacket(host_ip, localUdpPort);
    // udp.write((const uint8_t*)replyPacket, strlen(replyPacket));
    // udp.endPacket();
    // Serial.printf("Sent %s to %s:%d\n", replyPacket, udp.remoteIP().toString().c_str(), udp.remotePort());
}

void recvAbtr()
{
    // int len = udp.read(incomingPacket, 255);
    // if (len > 0)
    // {
    //     incomingPacket[len] = 0; // Null-terminate the string
    // }
    // Serial.printf("Received %d bytes: %s\n", len, incomingPacket);
    // led_blink = !led_blink;
    // digitalWrite(LED_BUILTIN, led_blink);
    int packetSize = udp.parsePacket();

    if (packetSize)
    {
        Serial.printf("Packet received, size: %d\n", packetSize);
        int len = udp.read(incomingPacket, 255);
        if (len > 0)
        {
            incomingPacket[len] = 0; // Null-terminate the string
        }
        Serial.printf("Received %d bytes: %s\n", len, incomingPacket);
        led_blink = !led_blink;
        digitalWrite(LED_BUILTIN, led_blink);
    }
    else
    {
        Serial.println("No packet received");
    }

    if (timerSend.isDone())
    {
        udp.beginPacket(host_ip, localUdpPort);
        udp.write((const uint8_t *)replyPacket, strlen(replyPacket));
        udp.endPacket();
        Serial.printf("Sent %s to %s:%d\n", replyPacket, udp.remoteIP().toString().c_str(), udp.remotePort());
    }
    // if (incomingPacket == "0.00")
    // {
    //     led_blink = !led_blink;
    //     digitalWrite(LED_BUILTIN, led_blink);
    // }
    // }
}
