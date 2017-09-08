#include <SoftwareSerial.h>

SoftwareSerial mySerial(9, 8); //RX, TX

void setup() {

  digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
  mySerial.begin(9600);
  Serial.begin(9600);
  delay(200);                       // wait for a second
  digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
}
void loop() {
  if (mySerial.available()) {
    while(mySerial.available()) {  
      char c = mySerial.read();
      Serial.print(c); 
    }
  }
  delay(50); //allows all serial sent to be received together  
}
