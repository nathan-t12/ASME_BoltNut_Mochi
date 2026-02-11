#include <Arduino.h>
  int a=1;
  int b= 2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);


}

void loop() {
  Serial.println("Hello World");
  Serial.println(a+b);
}

