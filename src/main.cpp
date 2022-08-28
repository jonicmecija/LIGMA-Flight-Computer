/*!
 *  @file main.cpp
 *
 *  This file contains the main logic for the LIGMA flight computer.
 *
 *  Uses a state machine for data acquisition and dual-parachute deployment.
 * 
 *  By: Jonic Mecija
 *  Date: 8/28/22
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

enum RocketStates
{
  IDLE_STATE,
  ASCENT_STATE,
  DESCENT_STATE,
  RECOVERY_STATE
};

// Initial state set to IDLE_STATE
RocketStates currState = IDLE_STATE;
Adafruit_BMP280 bmp(BMP_CS); // hardware SPI

// set global variable to keep track of altitude
float currAltitude = 0;
float initialAltitude = 0;

// time variables 
unsigned long prevMillis = 0;
unsigned long currMillis = 0;
int interval = 0;

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
  
  initialAltitude = bmp.readAltitude(1013.25);
  currAltitude = initialAltitude;
}

void loop() {
  currMillis = millis();
  if ( currMillis - prevMillis > 1000){
    prevMillis = currMillis;
    Serial.print("Current milliseconds: ");
    Serial.println(currMillis);
  
  switch(currState){
    case RocketStates::IDLE_STATE:
      Serial.println("idle state");
      
      Serial.print("Current Altitude: ");
      Serial.print(currAltitude);
      Serial.println(" m");

      Serial.print("Initial Altitude: ");
      Serial.print(initialAltitude);
      Serial.println(" m");
      Serial.println(" ");

      // if change in z axis altimeter has changed dramatically, change state
      if (currAltitude - initialAltitude > 0.3){
        Serial.println("Launch detected. Changing state.");
        currState = RocketStates::ASCENT_STATE;
      }
      break;

    case RocketStates::ASCENT_STATE:
      Serial.println("ascent state");
      // if rocket is descending, change state
      // currState = Rocket_States::DESCENT_STATE;
      break;
      
    case RocketStates::DESCENT_STATE:
      Serial.println("descent state");
      // when rocket hits certain altitude, fire main parachute
      // currState = Rocket_States::RECOVERY_STATE;
      break;
    case RocketStates::RECOVERY_STATE:
      Serial.println("recovery state");
      // currState = Rocket_States::IDLE_STATE;
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
  currAltitude = bmp.readAltitude(1013.25);
  // Serial.println(" m");
  // Serial.println();
  }
  // delay(500);
}