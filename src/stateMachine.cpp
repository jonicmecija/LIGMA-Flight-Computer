
/*!
 *  @file stateMachine.cpp
 *
 *  State machine library for the LIGMA rocket flight computer.
 *
 *  Jonic Mecija
 *  November 6, 2022
 */

#include <Arduino.h>

/* State Transition Functions */
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