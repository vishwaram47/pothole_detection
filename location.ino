
#include <SoftwareSerial.h>

// The serial connection to the GPS module
SoftwareSerial ss(3, 4);
void setup(){
  Serial.begin(9600);
  ss.begin(9600);
}

void loop(){
  while (ss.available() > 0){
    byte gpsData = ss.read();
    Serial.write(gpsData);
  }
}
