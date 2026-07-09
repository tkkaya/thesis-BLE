#include <Arduino.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("HELLO");
}

void loop() {
  Serial.println("ALIVE");
  delay(1000);
}