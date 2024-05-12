#include <SoftwareSerial.h>
// Pin10接HC05的TXD
// Pin1接HC05的RXD
SoftwareSerial BT(1,0);
char val;

void setup() {
  Serial.begin(9600);
  Serial.println("bluetooth is ready!");
  BT.begin(9600);
}

void loop() {
//  if (Serial.available()) {
//    val = Serial.read();
//    BT.print(val);
//  }
//
//  if (BT.available()) {
//    val = BT.read();
//    Serial.print(val);
//  }
Serial.println("鲍逸君好帅");

delay(100);
Serial.println("范訸然好帅");
delay(100);
Serial.println("姚亦珂好帅");
delay(100);
}
