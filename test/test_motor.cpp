#include <Arduino.h>

// Simple motor‑test sketch
// Pin D5 (GPIO14 on ESP8266 NodeMCU) drives the transistor base

const int motorPin = D5;

void setup() {
  Serial.begin(115200);
  pinMode(motorPin, OUTPUT);
  digitalWrite(motorPin, LOW);  // ensure motor is off at start
}

void loop() {
  Serial.println("ON");
  digitalWrite(motorPin, HIGH); // transistor saturates → motor on
  delay(200);                    // motor runs 200 ms

  Serial.println("OFF");
  digitalWrite(motorPin, LOW);  // motor off
  delay(800);                   // wait 800 ms before next buzz
}
