#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP280.h>
#include <Arduino.h>
#include <SPI.h>

/* BNO055 Connections for Teensy 4.1

   Connect SCL to pin 19
   Connect SDA to pin 18
   Connect VDD to 3.3V DC
   Connect GROUND to common ground
*/

enum States{
  IDLE = 0,
  ASCENT = 1,
  DESCENT = 2,
  RECOVERY = 3
};

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

/* BMP 280 SPI Connections */
#define BMP_SCK  (13)
#define BMP_MISO (12)
#define BMP_MOSI (11)
#define BMP_CS   (10)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                  id, address
Adafruit_BNO055 bno(-1, 0x28);
Adafruit_BMP280 bmp(BMP_CS); // hardware SPI

float thetaG = 0;
float phiG = 0;
float thetaA = 0;
float phiA = 0;
float theta = 0;
float phi = 0;
float dt = 0;
unsigned long prevMillis;
uint8_t currState;

// Detect Launch vars
int setAltitudeFlag = LOW;
float thresholdAltitude = 0.20;
float initialPressure = 0;
float initialAltitude = 0;
int currAltitude = 0;
int prevAltitude = 0;
 int counter = 0;
// Function Declarations
bool detectLaunch(float currAltitude);
bool detectApogee(float currAltitude, float prevAltitude);
bool detectLanding(float currAltitude);
void writeFile();
void printData(Adafruit_BNO055& bno, Adafruit_BMP280& bmp);

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  Serial.println(F("BMP280 test"));

  //  if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
   if (!bmp.begin()) {
     Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                       "try a different address!"));
     while (1) delay(10);
   }

   /* Default settings from datasheet. */
   bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                   Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                   Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                   Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                   Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);
  bno.setMode(OPERATION_MODE_ACCGYRO);

  // need to create state machine object, also keeps track of currState
  currState = States::IDLE;

  // Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");
  delay(1000);
  prevMillis = millis();
 
}

void loop(void)
{
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2

  printData(bno, bmp);
  delay(BNO055_SAMPLERATE_DELAY_MS);

  switch(currState)
  {
    case States::IDLE:
    {
      // need to read sensor information from BPM280 altitude
      // initialAltitude = bmp.readAltitude(1013.25)

      counter += 1;
      float currPressure = bmp.readPressure() /100;
      float currAltitude = bmp.readAltitude(currPressure);
      if (counter > 10){
        currState = (detectLaunch(currAltitude)) ? States::ASCENT : States::IDLE;
      }
      
      break;
    }
    case States::ASCENT:
    {
      Serial.println("in ascent state");
      // float prevAltitude;
      // currState = (detectApogee(currAltitude, prevAltitude)) ? States::ASCENT : States::IDLE;
      break;
    }
    case States::DESCENT:
    {
      // detectLanding(currAltitude);
      break;
    }
    case States::RECOVERY:
    {
      // writeFile();      
      break;
    }
    default:
    {
      break;
    }
  }

}

void printData(Adafruit_BNO055 &bno, Adafruit_BMP280& bmp){
  
  // grab raw accel and gyro values
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

  dt = (millis() - prevMillis) / 1000.0; // change to millis
  prevMillis = millis();

  thetaA = atan2(accel.x()/9.8, accel.z()/9.8)/2/3.141592654*360; // pitch from acceleration vector
  phiA = atan2(accel.y()/9.8, accel.z()/9.8)/2/3.141592654*360;   // roll from acceleration vector

  thetaG = thetaG - gyro.y() * dt; // pitch from gyro vector
  phiG = phiG + gyro.x() * dt;     // roll from gyro vector

  theta = (theta -  gyro.y() * dt) * 0.90 + thetaA * 0.1;
  phi = (phi + gyro.x() * dt) * 0.90 + phiA * 0.1;

  /* Display the floating point data */
  Serial.print(-gyro.y());
  Serial.print(",");
  Serial.print(gyro.x());
  Serial.print(",");
  Serial.print(accel.x()/9.8);
  Serial.print(",");
  Serial.print(accel.y()/9.8);
  Serial.print(",");
  Serial.print(accel.z()/9.8);

  Serial.print(",");
  Serial.print(thetaG);
  Serial.print(",");
  Serial.print(phiG);

  Serial.print(",");
  Serial.print(thetaA);
  Serial.print(",");
  Serial.print(phiA);
  Serial.print(",");
  Serial.print(theta);
  Serial.print(",");
  Serial.print(phi);
  // Serial.println("");

  /* Display calibration status for each sensor. */
  // uint8_t system, gyro, accel, mag = 0;
  // bno.getCalibration(&system, &gyro, &accel, &mag);
  // Serial.print("CALIBRATION: Sys=");
  // Serial.print(system, DEC);
  // Serial.print(" Gyro=");
  // Serial.print(gyro, DEC);
  // Serial.print(" Accel=");
  // Serial.print(accel, DEC);
  // Serial.print(" Mag=");
  // Serial.println(mag, DEC);

  Serial.print(",");
  Serial.print(bmp.readTemperature());

  Serial.print(",");
  Serial.print(bmp.readPressure()/100);
  Serial.print(",");

  Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
  // currAltitude = bmp.readAltitude(1013.25);

  Serial.println();

}

bool detectLaunch(float currAltitude){
  // blink LED every 0.5 seconds to show its in armed state
  
  // if initial altitude has not been set, set it then trigger flag
  Serial.print("set altitude flag: ");
  Serial.println(setAltitudeFlag);
  if (setAltitudeFlag == LOW){

    // initialPressure = bmp.readPressure() / 100;
    initialAltitude = bmp.readAltitude(1013.25);
    setAltitudeFlag = HIGH;
    // Serial.println(setAltitudeFlag);
    Serial.print("initial altitude set @ ");
    Serial.print(initialAltitude);
    Serial.println(" meters");
  }
  // if initial altitude is greater than -100 AND diff between currAlt and startingAlt is greater than threshold, you launched
  else if (initialAltitude > -100){
    currAltitude = bmp.readAltitude(1013.25) - initialAltitude;

    Serial.println("** detect launch variables **");
    Serial.print("Initial Altitude: "); Serial.println(initialAltitude);
    // currAltitude = bmp.readAltitude()
    Serial.print("Current Altitude: "); Serial.println(currAltitude);

    if( currAltitude > thresholdAltitude){
      Serial.println("Launch detected");
      return true;
    }
  }
  return false;
}

bool detectApogee(float currAltitude, float prevAltitude){
  // if decreasing after launch, apogee is reached or rocket went wrong
  Serial.println("rocket ascending");
  if (currAltitude > prevAltitude){

    // fire parachute
    // turn pin on
    Serial.println("Apogee detected. Firing Parachutes.");
    return true;
  }

  return false;
}

bool detectLanding(float currAltitude){

  // if altitude is the same alti tude for like 10 seconds you landed

  Serial.println("Landing Detected");
  return false;

}

void writeFile(){

  Serial.println("Writing to file");
}
