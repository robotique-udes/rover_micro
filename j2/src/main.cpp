#include <Arduino.h>
#include <J2Controller.hpp>

J2Controller j2Controller;

void setup()
{
  Serial.begin(115200);  // Start Serial Monitor
  Serial.println("Main setup started");
}


void loop()
{
  int32_t desiredRPM = 1; // Example RPM
  j2Controller.sendSpeedCommand(desiredRPM);
  //j2Controller.readMotorParameters();
  delay(100);
}
