#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <StateMachine.h>

/* This driver reads raw data from the BNO055

   Connections for Teensy 4.1
   ===========
   Connect SCL to pin 19
   Connect SDA to pin 18
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x28);

float thetaG = 0;
float phiG = 0;
float thetaA = 0;
float phiA = 0;
float theta = 0;
float phi = 0;

float dt = 0;
unsigned long prevMillis;
States currState;

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

  delay(1000);

  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  Serial.print("Current Temperature: ");
  Serial.print(temp);
  Serial.println(" C");
  Serial.println("");

  bno.setExtCrystalUse(true);
  bno.setMode(OPERATION_MODE_ACCGYRO);
  currState = States::IDLE;

  // Serial.println("Calibration status values: 0=uncalibrated, 3=fully calibrated");

  prevMillis = millis();
}

void loop(void)
{
  switch(currState)
  {
    case States::IDLE:
    {
      // code
      break;
    }
    case States::ASCENT:
    {
      // code
      break;
    }
    case States::DESCENT:
    {
      // code
      break;
    }
    case States::RECOVERY:
    {
      // code
      break;
    }
    default:
    {
      break;
    }
  }
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2

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
  Serial.println("");

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

  delay(BNO055_SAMPLERATE_DELAY_MS);
}