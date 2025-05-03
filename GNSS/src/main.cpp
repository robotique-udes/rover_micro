#include <Arduino.h>
#include <GNSSManager.hpp>

// Configure UART pins
#define UART_TX_PIN 47
#define UART_RX_PIN 13
#define UART_BAUD_RATE 921600
// Initialize HardwareSerial on UART2
HardwareSerial GNSSSerial(2); 
GNSSManager gnss(&GNSSSerial);


void setup()
{
  Serial.begin(115200);  // Start Serial Monitor
  Serial.println("Main setup started");

  GNSSSerial.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
}


void loop()
{
  gnss.update();
  GNSSData data = gnss.getData();

  if (data.validFix) {
      Serial.printf("Lat: %.6f, Lon: %.6f, Satellites: %d\n", data.latitude, data.longitude, data.satellites);
  } else {
      Serial.println("Waiting for fix...");
  }

  delay(500);
}



// https://receiverhelp.trimble.com/alloy-gnss/en-us/nmea0183-messages-gga.html?Highlight=gpgga