#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <driver/gpio.h>
#include "rover_lib2/helpers/log.hpp"

DEFINE_LOG_NODE(Main, Logger::eNodeState::ON);

// UART Configuration
static constexpr uint8_t UART_NUM = 2U;
static constexpr gpio_num_t RX_PIN = GPIO_NUM_4;
static constexpr gpio_num_t TX_PIN = GPIO_NUM_5;
static constexpr uint32_t UART_BAUD_RATE = 115200UL;
static constexpr size_t UART_BUFFER_SIZE = 256UL;

// WiFi Configuration
static const char* const WIFI_SSID = "67-65-69-67-65-72";
static const char* const WIFI_PASSWORD = "ske_bou_bou";
static constexpr uint32_t WIFI_TIMEOUT_MS = 10'000UL;

// UDP Configuration
static const char* const UDP_ADDRESSES[] = {
    "192.168.144.20",   // Rover MCU
    "192.168.144.101",  // Phil Michaud
    "192.168.144.110",  // Ethan Beaudoin
    "192.168.144.111"   // Émile Savoie
};
static constexpr size_t UDP_ADDRESS_COUNT = sizeof(UDP_ADDRESSES) / sizeof(UDP_ADDRESSES[0]);
static constexpr uint16_t UDP_PORT = 5005;
static constexpr size_t MAX_LINE_LENGTH = 512;

// Global objects
HardwareSerial geigerSerial(UART_NUM);
WiFiUDP udp;
String lineBuffer;

void setupWiFi()
{
    LOG_INFO(Logger::Nodes::Main, "Connecting to WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    LOG_INFO(Logger::Nodes::Main, "Connecting to %s\n", WIFI_SSID);

    uint32_t startTime = millis();
    while (WiFi.status() != WL_CONNECTED)
    {
        if (millis() - startTime > WIFI_TIMEOUT_MS)
        {
            LOG_ERROR(Logger::Nodes::Main, "\nWiFi connection failed, status: %d\n", WiFi.status());
            ESP.restart();
        }
        delay(500);
        LOG_INFO(Logger::Nodes::Main, ".");
    }

    LOG_INFO(Logger::Nodes::Main, "\nWiFi connected! IP: %s\n", WiFi.localIP().toString().c_str());
    LOG_INFO(Logger::Nodes::Main, "Signal strength: %d dBm\n", WiFi.RSSI());
}

void setupUART()
{
    geigerSerial.begin(UART_BAUD_RATE, SERIAL_8N1, RX_PIN, TX_PIN);
    geigerSerial.setRxBufferSize(UART_BUFFER_SIZE);
    LOG_INFO(Logger::Nodes::Main,
             "UART%d initialized on pins RX:%d, TX:%d at %lu baud\n",
             UART_NUM,
             RX_PIN,
             TX_PIN,
             UART_BAUD_RATE);
}

void setup()
{
    Serial.begin(115'200);
    while (!Serial && millis() < 5'000)
        ;  // Wait up to 5s for serial

    LOG_INFO(Logger::Nodes::Main, "ESP32 UART to UDP Bridge Starting...");

    // Reserve buffer space to prevent fragmentation
    lineBuffer.reserve(MAX_LINE_LENGTH);

    setupUART();
    setupWiFi();

    LOG_INFO(Logger::Nodes::Main, "Setup complete. Ready to forward UART data to UDP.");
}

inline void sendUDPPacket(const String& data)
{
    if (WiFi.status() == WL_CONNECTED)
    {
        for (size_t i = 0; i < UDP_ADDRESS_COUNT; i++)
        {
            udp.beginPacket(UDP_ADDRESSES[i], UDP_PORT);
            udp.write(reinterpret_cast<const uint8_t*>(data.c_str()), data.length());
            if (udp.endPacket())
            {
                LOG_INFO(Logger::Nodes::Main, "UDP sent to %s (%d bytes): %s\n", UDP_ADDRESSES[i], data.length(), data.c_str());
            }
            else
            {
                LOG_WARN(Logger::Nodes::Main, "UDP send failed to %s\n", UDP_ADDRESSES[i]);
            }
        }
    }
    else
    {
        LOG_ERROR(Logger::Nodes::Main, "WiFi disconnected - cannot send UDP packet");
        WiFi.reconnect();
    }
}

void processUARTData()
{
    while (geigerSerial.available())
    {
        char c = geigerSerial.read();

        if (c == '\n')
        {
            lineBuffer.trim();  // Remove any trailing whitespace/carriage returns

            if (lineBuffer.length() > 0)
            {
                sendUDPPacket(lineBuffer);
                lineBuffer.clear();
            }
        }
        else if (c != '\r')
        {  // Skip carriage returns
            lineBuffer += c;

            // Prevent buffer overflow
            if (lineBuffer.length() >= MAX_LINE_LENGTH)
            {
                LOG_WARN(Logger::Nodes::Main, "Warning: Line buffer overflow, sending partial data");
                sendUDPPacket(lineBuffer);
                lineBuffer.clear();
            }
        }
    }
}

void loop()
{
    processUARTData();

    // Handle WiFi reconnection if needed
    static uint32_t lastWiFiCheck = 0;
    if (millis() - lastWiFiCheck > 30000)
    {  // Check every 30 seconds
        if (WiFi.status() != WL_CONNECTED)
        {
            LOG_WARN(Logger::Nodes::Main, "WiFi disconnected, attempting reconnection...");
            WiFi.reconnect();
        }
        lastWiFiCheck = millis();
    }
    // Small delay to prevent watchdog issues and reduce power consumption
    delay(1);
}
