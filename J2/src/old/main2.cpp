// #include <Arduino.h>
// #include "J2Controller.hpp"

// // Configure UART pins
// #define UART_TX_PIN 6
// #define UART_RX_PIN 7
// #define UART_BAUD_RATE 921600
// // Initialize HardwareSerial on UART2
// HardwareSerial motorSerial(2);
// J2Controller controller(&motorSerial);

// float desiredRPM = 0.0;
// float vitesse = 2.5;

// void setup()
// {
//     Serial.begin(115200);  // Start Serial Monitor
//     Serial.println("Main setup started");

//     motorSerial.begin(UART_BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
// }

// void loop()
// {
//     controller.setSpeed(0.0);
//     if (controller.isJogButtonPressed(0))
//     {
//         controller.setSpeed(-vitesse);
//     }

//     if (controller.isJogButtonPressed(1))
//     {
//         controller.setSpeed(vitesse);
//     }

//     controller.update();
//     // controller.readMotorParameters(true);
// }
