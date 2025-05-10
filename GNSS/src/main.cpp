#include <Arduino.h>
#include <GNSSManager.hpp>

// Configure UART pins
#define UART_TX_PIN 47
#define UART_RX_PIN 13
#define UART_BAUD_RATE 921600


void setup()
{
  Serial.begin(115200);  // Start Serial Monitor
  Serial.println("Main setup started");

  // Initialize HardwareSerial on UART2
  HardwareSerial GNSSSerial(2); 
  GNSSManager gnss(&GNSSSerial);

  GNSSSerial.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);


  for (EVER)
  {
    gnss.update();
    GNSSData data = gnss.getData();

    if (data.hasValidFix()) {
      Serial.printf("Lat: %.6f, Lon: %.6f, Heading: %f deg, Quality: %d, Satellites: %d\n",
        data.latitude, data.longitude, data.headingDeg, data.fixQuality, data.satellites);
    } else {
        Serial.println("Waiting for a valid fix...");
    }

    delay(500);
  }
}


void loop()
{
  
}



// https://receiverhelp.trimble.com/alloy-gnss/en-us/nmea0183-messages-gga.html?Highlight=gpgga// https://receiverhelp.trimble.com/alloy-gnss/en-us/nmea0183-messages-gga.html?Highlight=gpgga