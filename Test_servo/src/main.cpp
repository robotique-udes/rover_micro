#include <Arduino.h>

#define EN 27
#define DIR 26
#define PUL 25

void tickStepper(uint32_t freq);

void setup() {
  Serial.begin(9600);
  pinMode(EN, OUTPUT);
  pinMode(DIR, OUTPUT);
  pinMode(PUL, OUTPUT);

  digitalWrite(EN, LOW);
  digitalWrite(DIR, HIGH);
  // while (true)
  // {
  //   tickStepper(200);
  //   //Serial.println("tick done");
  // }
  
}

void loop() {
//   // digitalWrite(PUL, HIGH);
//   // delayMicroseconds(200);
//   // digitalWrite(PUL, LOW);
  tickStepper(200);
}

void tickStepper(uint32_t freq){
    digitalWrite(PUL,HIGH);
    delayMicroseconds(freq);
    digitalWrite(PUL,LOW);
}

