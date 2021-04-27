#include <SoftwareSerial.h>
SoftwareSerial HC12(11, 12); // HC-12 TX Pin, HC-12 RX Pin

void setup() {
  Serial.begin(9600);             // Serial port to computer
  HC12.begin(9600);               // Serial port to HC12
  pinMode(7,OUTPUT);
}
void loop() {
  while (HC12.available()) {        // If HC-12 has data
    String data = HC12.readStringUntil('\n');  //conveting the value of chars to integer
    if (data.startsWith("SL-1")){
    Serial.println(data);
    digitalWrite(7,HIGH);
    delay(200);
    digitalWrite(7,LOW);
    //Serial.println(HC12.read());      // Send the data to Serial monitor
  }
  }
  while (Serial.available()) {      // If Serial monitor has data
    HC12.write(Serial.read());      // Send that data to HC-12
  }
}
