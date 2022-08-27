#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

// Initial state set to IDLE_STATE
Rocket_States curr_state = IDLE_STATE;
Adafruit_BMP280 bmp(BMP_CS); // hardware SPI

enum Rocket_States
{
    IDLE_STATE,
    ASCENT_STATE,
    DESCENT_STATE,
    RECOVERY_STATE
};

void setup() {
  Serial.begin(9600);
  Serial.println("Program start");
  
  if (!bmp.begin()) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  Serial.println("");
  delay(100);
  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
}

void loop() {
  switch(curr_state){
    case Rocket_States::IDLE_STATE:
      Serial.println("idle state");
      curr_state = Rocket_States::ASCENT_STATE;
      break;
    case Rocket_States::ASCENT_STATE:
      Serial.println("ascent state");
      curr_state = Rocket_States::DESCENT_STATE;
      break;
    case Rocket_States::DESCENT_STATE:
      Serial.println("descent state");
      curr_state = Rocket_States::RECOVERY_STATE;
      break;
    case Rocket_States::RECOVERY_STATE:
      Serial.println("recovery state");
      curr_state = Rocket_States::IDLE_STATE;
      break;
  }
  
  /* BMP280 Sensor Readings */
  // Serial.print(F("Temperature = "));
  // Serial.print(bmp.readTemperature());
  // Serial.println(" *C");
  // Serial.print(F("Pressure = "));
  // Serial.print(bmp.readPressure());
  // Serial.println(" Pa");
  // Serial.print(F("Approx altitude = "));
  // Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  // Serial.println(" m");
  // Serial.println();

  delay(2000);
}