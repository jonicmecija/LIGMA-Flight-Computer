#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_BMP280.h>
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <cmath>

#define DEBUG_MODE 1 // do #if before every serial.print
                     // then set this to 0 to disable the serial port.

enum class States{
  IDLE = 0,
  ASCENT = 1,
  DESCENT = 2,
  RECOVERY = 3
};

/* Set the delay between fresh samples */
const int chipSelect = BUILTIN_SDCARD;
const char* fileName = "testFileName3.csv";
#define LOGS_PER_SECOND 10 

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
uint32_t prevMillis = 0;
uint32_t currMillis = 0;
uint32_t initMillis = 0;
States currState;

// Filter vars
float currAltitude_emaFiltered = 0;
static uint16_t t_currAltitude_ema = 1; // data point

// Detect Launch vars
int setAltitudeFlag = LOW;
float thresholdAltitude = 0.20;
float initialPressure = 0;
float initialAltitude = 0;
int prevAltitude = 0;
int counter = 0;

//detect apogee vars
float maxAltitude = 0;
float apogeeDetectAltitude = 0.2;

//detect landing
int landCounter = 0;

// Function Declarations
bool detectLaunch(float currAltitude);
bool detectApogee(float currAltitude);
bool detectLanding(float currAltitude);
void EMA1(float& ema, float newData, float smoothing, uint16_t& t_ema);
void EMA1(float& ema, float newData, uint16_t timePeriod, uint16_t& t_ema);
void DEMA(float& dema, float newData);
void writeFile();
void printData(Adafruit_BNO055& bno, Adafruit_BMP280& bmp);


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

  SD.begin(chipSelect);
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
  currMillis = millis();
  prevMillis = currMillis;
  initMillis = currMillis;
}

void loop(void)
{
  float currAltitude = bmp.readAltitude(1013.25) - initialAltitude;
  if(currAltitude > maxAltitude){
    maxAltitude = currAltitude;
  }

#if DEBUG_MODE  
  Serial.print("max altitude: ");
  Serial.println(maxAltitude);
  Serial.print("current altitude: ");
  Serial.println(currAltitude);
#endif

  //////////////////////////
  // Data filtering
  
  EMA1(currAltitude_emaFiltered, currAltitude, 0.9f, t_currAltitude_ema);  
  //EMA1(currAltitude_emaFiltered, currAltitude, static_cast<uint16_t>(3), t_currAltitude_ema);  // Omegalul
#if DEBUG_MODE
  Serial.print("current filtered altitude: ");
  Serial.println(currAltitude_emaFiltered);
#endif

  /////////////////////////

  /* STATE MACHINE */
  switch(currState)
  {
    case States::IDLE:
    {
      Serial.println("IDLE STATE");
      // counter to read first read first 10 readings of barometric pressure sensor
      counter += 1;
      if (counter > 10){
        currState = (detectLaunch(currAltitude)) ? States::ASCENT : States::IDLE;
      }
      break;
    }
    case States::ASCENT:
    {
      Serial.println("ASCENT STATE");
      currState = (detectApogee(currAltitude)) ? States::DESCENT : States::ASCENT;
      break;
    }
    case States::DESCENT:
    {
      Serial.println("DESCENT STATE");
      currState = (detectLanding(currAltitude)) ? States::RECOVERY : States::DESCENT;
      break;
    }
    case States::RECOVERY:
    {
      Serial.println("RECOVERY STATE");
      // writeFile();      
      // blink LED slowly for recovery state
      break;
    }
    default:
    {
      break;
    }
  }
  if(currMillis - prevMillis >= 1000.00/LOGS_PER_SECOND){
    // Serial.print((currTime-initialTime)/1000.00);
    // Serial.print(" ");
    // Serial.println(maxthermo.readThermocoupleTemperature());
    printData(bno, bmp);
    prevMillis = currMillis;
  }
  currMillis = millis();

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

  String datastring = "";
  datastring += String((currMillis-initMillis)/1000.00);
  datastring += ",";
  datastring += String(bmp.readTemperature());  

  File dataFile = SD.open(fileName, FILE_WRITE);
  
  if (dataFile) {
    dataFile.println(datastring);
    dataFile.close();  
    // Serial.println(datastring);
  }else {
    // Serial.println(fileName);
  }

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
  Serial.println();

}

bool detectLaunch(float currAltitude){
  // blink LED every 0.5 seconds to show its in armed state
  
  // if initial altitude has not been set, set it then trigger flag
  if (setAltitudeFlag == LOW){
    initialAltitude = bmp.readAltitude(1013.25);
    setAltitudeFlag = HIGH;
    Serial.print("initial altitude set @ ");
    Serial.print(initialAltitude);
    Serial.println(" meters");
  }

  // if initial altitude is greater than -100 AND diff between currAlt and startingAlt is greater than threshold, you launched
  else if (initialAltitude > -100){
    currAltitude = bmp.readAltitude(1013.25) - initialAltitude;

    if( currAltitude > thresholdAltitude){
      Serial.println("Launch detected");
      return true;
    }
    
    Serial.println("** detect launch variables **");
    Serial.print("Initial Altitude: "); Serial.println(initialAltitude);
    Serial.print("Current Altitude: "); Serial.println(currAltitude);
  }
  return false;
}

bool detectApogee(float currAltitude){
  // if decreasing after launch, apogee is reached or rocket went wrong
  // set max altitude
  if (maxAltitude - currAltitude > apogeeDetectAltitude){
    Serial.print("max altitude: ");
    Serial.println(maxAltitude);
    Serial.print("curr altitude: ");
    Serial.println(currAltitude);
    Serial.print("max - curr: ");
    Serial.println(maxAltitude - currAltitude);

    // fire parachute, turn pin on
    Serial.println("Apogee detected. Firing Parachutes.");
    return true;
  }

  return false;
}

bool detectLanding(float currAltitude){
  // if altitude is the same altitude for 10 seconds, rocket landed

  // currAltitude has been less than 10m for 20 counts
  if (landCounter > 20){
    Serial.println("Landing Detected!");
    return true;
  }

  // if curr altitude is less than 0.5m, wait till counter gets to 20
  else if (currAltitude < 0.05){
    Serial.print("count: ");
    Serial.println(landCounter);
    landCounter += 1;
  }
  return false;
}

void EMA1(float& ema, float newData, float smoothing, uint16_t& t_ema)
{
  /*
  Params:
  float& ema:       Exponential moving average value computed repeatedly.
  float newData:    New data coming from sensor.
  float smoothing:  Smoothing constant factor -- 0 < beta < 1.
                    The higher this is, the more weight is given to the 
                    moving average and vice versa for the new data coming in.
  uint16_t& t_ema:  Current data point.


  Return via reference:
  float& ema: this value is computed and updated repeatedly in the void loop()
  */
  ema = (smoothing*ema + (1-smoothing)*newData)/(1-pow(smoothing, t_ema));
  t_ema = t_ema + 1;
  /*
  Notes:
    -Bias Correction?

  */
}

void EMA1(float& ema, float newData, uint16_t timePeriod, uint16_t& t_ema)
{
  /*
  
  Unlike the other EMA1 function, this function lets you determine your own time window
  from which the smoothing constant is computed.

  Params:
  float& ema:           Exponential moving average value computed repeatedly
  float newData:        New data coming from sensor
  uint16_t timePeriod:  Time window of EMA. for eg., an 8-point time window
                        will have a smoothing constant of 0.875.
  uint16_t& t_ema:      Current data point.

  Return via reference:
  float& ema: this value is computed and updated repeatedly in the void loop()
  
  */
  float smoothing = 1 - (1/timePeriod);
  ema = (smoothing*ema + (1-smoothing)*newData)/(1-pow(smoothing, t_ema));
  t_ema = t_ema + 1;
}

void DEMA(float& dema, float newData)
{
    /*
  Params:
  float& ema: Exponential moving average value computed repeatedly
  float newData: new data coming from sensor
  float smoothing: smoothing constant factor -- 0 < beta < 1
  */
  
  ;

}
