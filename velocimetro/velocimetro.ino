# include <SoftwareSerial.h>
# include <TinyGPS++.h>

SoftwareSerial gpsSerial(10, 11); // RX, TX
TinyGPSPlus gps;

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
}

void loop() {
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.speed.isValid()) {
        float velocidad = gps.speed.kmph();
        if (velocidad > 25){
          Serial.print("Infracción, vas a ");
          Serial.print(velocidad);
          Serial.println(" km/h");

          Serial.print(gps.location.lat());
          Serial.print(", ");
          Serial.println(gps.location.lng());

          Serial.print("https://maps.google.com/?q=")
          Serial.print(gps.location.lat());
          Serial.print(",");
          Serial.print(gps.locaation.lng());
        }
        
      }
    }
  }
}