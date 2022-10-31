// #include <Wire.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
// #include <utility/imumaths.h>
// #include <Adafruit_BMP280.h>
// #include <Arduino.h>
// #include <SPI.h>

// /* BNO055 Connections for Teensy 4.1

//    Connect SCL to pin 19
//    Connect SDA to pin 18
//    Connect VDD to 3.3V DC
//    Connect GROUND to common ground
// */

// enum States{
//   IDLE = 0,
//   ASCENT = 1,
//   DESCENT = 2,
//   RECOVERY = 3
// };

// /* Set the delay between fresh samples */
// #define BNO055_SAMPLERATE_DELAY_MS (100)

// /* BMP 280 SPI Connections */
// #define BMP_SCK  (13)
// #define BMP_MISO (12)
// #define BMP_MOSI (11)
// #define BMP_CS   (10)

// // Check I2C device address and correct line below (by default address is 0x29 or 0x28)
// //                                   id, address
// Adafruit_BNO055 bno(-1, 0x28);


// // Adafruit_BMP280 bmp; // I2C
// // Adafruit_BMP280 bmp(BMP_CS, BMP_MOSI, BMP_MISO,  BMP_SCK);
// Adafruit_BMP280 bmp(BMP_CS); // hardware SPI

// float thetaG = 0;
// float phiG = 0;
// float thetaA = 0;
// float phiA = 0;
// float theta = 0;
// float phi = 0;
// float dt = 0;
// int currAltitude = 0;
// int prevAltitude = 0;
// unsigned long prevMillis;
// uint8_t currState;


// // Function Declarations
// bool detectLaunch(float currAltitude, float initialAltitude);
// bool detectApogee(float currAltitude, float prevAltitude);
// bool detectLanding(float currAltitude);
// void writeFile();
// void printData(Adafruit_BNO055& bno, Adafruit_BMP280& bmp);

// /**************************************************************************/
// /*
//     Arduino setup function (automatically called at startup)
// */
// /**************************************************************************/
// void setup(void)
// {
//   Serial.begin(115200);
//   Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");

//   /* Initialise the sensor */
//   if(!bno.begin())
//   {
//     /* There was a problem detecting the BNO055 ... check your connections */
//     Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
//     while(1);
//   }

//   Serial.println(F("BMP280 test"));

//   //  if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
//    if (!bmp.begin()) {
//      Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
//                        "try a different address!"));
//      while (1) delay(10);
//    }

//    /* Default settings from datasheet. */
//    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
//                    Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
//                    Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
//                    Adafruit_BMP280::FILTER_X16,      /* Filtering. */
//                    Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

//   delay(1000);

//   /* Display the current temperature */
//   int8_t temp = bno.getTemp();
//   Serial.print("Current Temperature: ");
//   Serial.print(temp);
//   Serial.println(" C");
//   Serial.println("");

//   bno.setExtCrystalUse(true);
//   bno.setMode(OPERATION_MODE_ACCGYRO);

//   // need to create state machine object, also keeps track of currState
//   currState = States::IDLE;

//   // Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

//   prevMillis = millis();
// }

// void loop(void)
// {
//   // Possible vector values can be:
//   // - VECTOR_ACCELEROMETER - m/s^2
//   // - VECTOR_MAGNETOMETER  - uT
//   // - VECTOR_GYROSCOPE     - rad/s
//   // - VECTOR_EULER         - degrees
//   // - VECTOR_LINEARACCEL   - m/s^2
//   // - VECTOR_GRAVITY       - m/s^2

//   printData(bno, bmp);
//   delay(BNO055_SAMPLERATE_DELAY_MS);

//   switch(currState)
//   {
//     case States::IDLE:
//     {
//       // need to read sensor information from BPM280 altitude
//       // float fakeVal1, fakeVal2;
//       // currState = (detectLaunch(fakeVal1, fakeVal2)) ? States::ASCENT : States::IDLE;
//       break;
//     }
//     case States::ASCENT:
//     {
//       // float prevAltitude;
//       // currState = (detectApogee(currAltitude, prevAltitude)) ? States::ASCENT : States::IDLE;
//       break;
//     }
//     case States::DESCENT:
//     {
//       // detectLanding(currAltitude);
//       break;
//     }
//     case States::RECOVERY:
//     {
//       // writeFile();      
//       break;
//     }
//     default:
//     {
//       break;
//     }
//   }

// }

// void printData(Adafruit_BNO055 &bno, Adafruit_BMP280& bmp){
  
//   // grab raw accel and gyro values
//   imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
//   imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);

//   dt = (millis() - prevMillis) / 1000.0; // change to millis
//   prevMillis = millis();

//   thetaA = atan2(accel.x()/9.8, accel.z()/9.8)/2/3.141592654*360; // pitch from acceleration vector
//   phiA = atan2(accel.y()/9.8, accel.z()/9.8)/2/3.141592654*360;   // roll from acceleration vector

//   thetaG = thetaG - gyro.y() * dt; // pitch from gyro vector
//   phiG = phiG + gyro.x() * dt;     // roll from gyro vector

//   theta = (theta -  gyro.y() * dt) * 0.90 + thetaA * 0.1;
//   phi = (phi + gyro.x() * dt) * 0.90 + phiA * 0.1;

//   /* Display the floating point data */
//   Serial.print(-gyro.y());
//   Serial.print(",");
//   Serial.print(gyro.x());
//   Serial.print(",");
//   Serial.print(accel.x()/9.8);
//   Serial.print(",");
//   Serial.print(accel.y()/9.8);
//   Serial.print(",");
//   Serial.print(accel.z()/9.8);

//   Serial.print(",");
//   Serial.print(thetaG);
//   Serial.print(",");
//   Serial.print(phiG);

//   Serial.print(",");
//   Serial.print(thetaA);
//   Serial.print(",");
//   Serial.print(phiA);
//   Serial.print(",");
//   Serial.print(theta);
//   Serial.print(",");
//   Serial.print(phi);
//   // Serial.println("");

//   /* Display calibration status for each sensor. */
//   // uint8_t system, gyro, accel, mag = 0;
//   // bno.getCalibration(&system, &gyro, &accel, &mag);
//   // Serial.print("CALIBRATION: Sys=");
//   // Serial.print(system, DEC);
//   // Serial.print(" Gyro=");
//   // Serial.print(gyro, DEC);
//   // Serial.print(" Accel=");
//   // Serial.print(accel, DEC);
//   // Serial.print(" Mag=");
//   // Serial.println(mag, DEC);

//   Serial.print(",");
//   Serial.print(bmp.readTemperature());
//   Serial.print(",");

//   Serial.print(",");
//   Serial.print(bmp.readPressure());
//   Serial.print(",");

//   Serial.print(",");
//   Serial.print(bmp.readAltitude(1013.25)); /* Adjusted to local forecast! */
//   // currAltitude = bmp.readAltitude(1013.25);

//   Serial.println();
// }
// bool detectLaunch(float currAltitude, float initialAltitude){
//   // if passed 10 meter threshold, then rocket has launched
//   if ((currAltitude - initialAltitude) > 10){
//     return true;
//   }

//   return false;
// }

// bool detectApogee(float currAltitude, float prevAltitude){
//   // if decreasing after launch, apogee is reached or rocket went wrong
//   if (currAltitude > prevAltitude){

//     // fire parachute
//     // turn pin on
//     Serial.println("Apogee detected. Firing Parachutes.");
//     return true;
//   }

//   return false;
// }

// bool detectLanding(float currAltitude){

//   // if altitude is the same altitude for like 10 seconds you landed

//   Serial.println("Landing Detected");
//   return false;

// }

// void writeFile(){

//   Serial.println("Writing to file");
// }

/***************************************************************************
  This is a library for the bmp280 humidity, temperature & pressure sensor
  Designed specifically to work with the Adafruit bmp280 Breakout
  ----> http://www.adafruit.com/products/2650
  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface. The device's I2C address is either 0x76 or 0x77.
  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!
  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor
  This example shows how to take Sensor Events instead of direct readings
  
  Designed specifically to work with the Adafruit BMP280 Breakout
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

void setup() {
  Serial.begin(9600);
  Serial.println(F("BMP280 Sensor event test"));

  //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
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

  bmp_temp->printSensorDetails();
}

void loop() {
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  
  Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  Serial.println(" *C");

  Serial.print(F("Pressure = "));
  Serial.print(pressure_event.pressure);
  Serial.println(" hPa");

  Serial.println();
  delay(2000);
}