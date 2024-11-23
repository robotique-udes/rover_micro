#include <Arduino.h>
#include <ESPTelnetStream.h>

ESPTelnetStream telnet;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  
  pinMode(LED_BUILTIN, OUTPUT);

  // Connect to WiFi
  const char *ssid = "ROVUS";
  const char *pass = "123456789";

  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi... ssid: " + String(ssid) + " pass: " + String(pass));
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
  }
  Serial.println("Connected to WiFi!");
  Serial.println("IP address: " + WiFi.localIP().toString());
  
  // Start Telnet server
  telnet.begin();
}

void loop() {
  telnet.loop();
  // Read from Serial2 and send to Serial and WiFi
  if (Serial2.available()) {
    char c = Serial2.read();
    Serial.write(c);
    telnet.write(c);
  }
  
  // Read from WiFi and send to Serial and Serial2
  if (telnet.available()) {
    char c = telnet.read();
    Serial.write(c);
    Serial2.write(c);
  }

  if (Serial.available()) {
    char c = Serial.read();
    Serial2.write(c);
    telnet.write(c);
  }
}