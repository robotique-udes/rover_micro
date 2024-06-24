#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>

// Set up the Wi-Fi network credentials for the ESP32 AP
const char* ssid = "roverAntenna";
const char* password = "roverAntenna";
IPAddress local_ip(192, 168, 144, 100);
IPAddress gateway(192, 168, 144, 1);
IPAddress subnet(255, 255, 255, 0);

WiFiUDP udp;
unsigned int localUdpPort = 1234;  // local port to listen on
char incomingPacket[255];  // buffer for incoming packets
const char* replyPacket = "pong";  // a reply string to send back

void setup() {
  Serial.begin(115200);

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

void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    // Receive incoming UDP packets
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;  // Null-terminate the string
    }
    Serial.printf("Received %d bytes: %s\n", len, incomingPacket);

    // Send a reply to the IP address and port that sent the incoming packet
    udp.beginPacket(udp.remoteIP(), udp.remotePort());
    udp.write((const uint8_t*)replyPacket, strlen(replyPacket));
    udp.endPacket();
    Serial.printf("Sent %s to %s:%d\n", replyPacket, udp.remoteIP().toString().c_str(), udp.remotePort());
  }
}
