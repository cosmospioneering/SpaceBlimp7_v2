


void calcActivateStabilizer(boolean useOneSecondTimer) {

  if ((useOneSecondTimer == true && lastStabilizerSecond !=now.second()) || useOneSecondTimer == false) {
    stabilizedCount++;
    if (abs(GyroTurnRate) > MAX_ACCEPTIBLE_SPIN && activeStabilize == false) {
      //If we are not yet actively stabilizing, but we need to turn it on, then do so!
      //Serial.print("Gyro-1\t");
      //Serial.println(GyroTurnRate);
      activeStabilize = true;
      //Reset the stabilizer count
      //stabilizedCount=0;
    } else if (activeStabilize == true && abs(GyroTurnRate) > TARGET_ACCEPTIBLE_SPIN) {
      //If we are actively stabilizing, but have not yet reduced it to an acceptible level, then leave it on!
      //Serial.print("Gyro-2\t");
      //Serial.println(GyroTurnRate);
      activeStabilize = true;
      //Reset the stabilizer count
      //stabilizedCount=0;
    } else {
      //We have achieved acceptible stabilization, we can stop now
      //Serial.print("Gyro-3\t");
      //Serial.println(GyroTurnRate);
      //
      activeStabilize = false;
      //Reset the stabilizer count
      stabilizedCount=0;
      
      //Have we been stablized long enough?
      //if (stabilizedCount > MIN_STABILIZED_COUNT) {
      //  //Yes!  Deactivate the stabilizer
      //} else {
        //Incerase the stabilizer count
      //  stabilizedCount++;
      //}
    }
    if (stabilizedCount > 5) { stabilizedCount = 0; }
    lastStabilizerSecond = now.second();
  }
  
}



void calcFloatStates() {
int intAltitude;


  if (pauseAltSensorReading == true ) {
    //We have our smoothed values from the first minute and ten minutes of flight
    //
    //So, we need to get a smoothed value from the accelerometer for the Ascent Rate
    //
    //
    //add in 130% of the calculated ascent rate
    //We do this because after some time, the ascent rate increases and we this gets
    //us closer to the expected altitude later on.
    if (ascendingFlag == true) {
      intAltitude = (int)(altDEMA + (firstTenMinutePerSecondAscentRate * 1.3));
    } else if ( floatingFlag == true) {
      intAltitude = (int)(altDEMA + (firstTenMinutePerSecondAscentRate * 2));
    } else if (decendingFlag == true) {
      if (firstTenMinutePerSecondAscentRate ==0) {firstTenMinutePerSecondAscentRate = firstMinutePerSecondAscentRate;}
      intAltitude = (int)(altDEMA - (firstTenMinutePerSecondAscentRate * 2));
    }
    ema = EMA_function(0.1, intAltitude, ema);
    ema_ema = EMA_function(0.1, ema, ema_ema);
  }
  
  
  //Finish calculating the altitude trends
  altLastDEMA = altDEMA;  //store the previous DEMA value
  altDEMA = abs(2*ema - ema_ema);  //Calculate the new DEMA value
  ascentRate = (altDEMA - altLastDEMA);

  if (altDEMA > 0 || pauseAltSensorReading == true ) {

    //manage the min/max values
    if (minDEMA == 0) { minDEMA = altDEMA; }
    if (altDEMA > 0 && altDEMA < minDEMA) { minDEMA = altDEMA; }
    if (altDEMA > maxDEMA) { maxDEMA = altDEMA; }

    
    Serial.print("DEMA:  ");
    Serial.print(intAltitude);
    Serial.print("    ");
    Serial.print(altDEMA);
    Serial.print("    ");
    Serial.print(altLastDEMA - altDEMA);
    Serial.print("    ");
    Serial.println(firstTenMinutePerSecondAscentRate);

/*
    Serial.print("Launch:  ");
    Serial.print(launchedFlag);
    Serial.print("    ");
    Serial.print(ascendingFlag);
    Serial.print("    ");
    Serial.print(decendingFlag);
    Serial.print("    ");
    Serial.print(floatingFlag);
    Serial.print("    ");
    Serial.println(pauseAltSensorReading);
*/ 
    

    //
    if (launchedFlag == false) {
      if (altLastDEMA > minDEMA && altDEMA > minDEMA && altDEMA > altLastDEMA && altDEMA - minDEMA > LAUNCH_DETECT_ALTITUDE) {
        launchedFlag = true;
        ascendingFlag = true;
        decendingFlag = false;
        //Get the launch altitude!
        launchDetectedAltitude = (int)altitude;
        //Get the launch minute!
        launchDetectedMinute = now.minute();
        //Get the launch second!
        launchDetectedSeconds = now.second();
        //calculate the ten minute detection point
        launchTenMinutePoint = launchDetectedMinute + 10;
        //Adjust for overlap
        if (launchTenMinutePoint >= 60) { launchTenMinutePoint = launchTenMinutePoint - 60; }
      }
    } else {



      //-----------------------------------------------------------------------------------------------------
      //This section will calculate the estimated values to be used later in the flight!
      if (firstMinuteFinalAltitude == 0 || firstTenMinutesFinalAltitude == 0) {

        //This section grabs the altitude value for each "second" during the first minute of flight
        if (firstMinuteFinalAltitude == 0 && lastAltitudeSampleSecond != now.second()) {
          //calculate the maximum allowed Estimated Accelerometer Per Second Ascent Rate,
          //Smooth the ascent rate measured in the first minute of flight
          //
          //The Ascent rate is how much distance we traveled between "readings" each second
          //maxAllowedPerSecondEstimatedAccelerometerAscentAmount = (int)((maxAllowedPerSecondEstimatedAccelerometerAscentAmount * 0.75) + (((int)altitude - lastAltitudeSample) * 0.25)) ;
          //
          //Store the current altitude as the last Altitude for the next smoothing cycle
          lastAltitudeSample = (int)altitude;
          //
          //We now have the samples for this "second"
          //
          //Set the variable to the current "second"
          //so we don't grab another value during the same "second"
          lastAltitudeSampleSecond = now.second();
        }
        
        //This section roughly calculates a value to be used to estimate the altitude over time, for the first minute of flight
        if (firstMinuteFinalAltitude == 0 && launchDetectedMinute != now.minute() && launchDetectedSeconds >= now.second()) {
          firstMinuteFinalAltitude = (int)altitude;
          firstMinutePerSecondAscentRate = (firstMinuteFinalAltitude - launchDetectedAltitude) / 60;
          //We'll use this to set the maximum allowed values from the accelerometer ascent estimator
          //maxAllowedPerSecondEstimatedAccelerometerAscentAmount += 1;
          //This value is our best, simplified guess at the ascent rate for the first minute of flight!
          //calcAccelerometerPerSecondAscentAmount = (uint8_t) firstMinutePerSecondAscentRate;
        }
        
       //This section roughly calculates a value to be used to estimate the altitude over time, for the first minute of flight
       if (firstTenMinutesFinalAltitude == 0 && launchTenMinutePoint == now.minute() && launchDetectedSeconds >= now.second()) {
          firstTenMinutesFinalAltitude = (int)altitude;
          //This value is our best, simplified guess at the ascent rate for the first minute of flight!
          firstTenMinutePerSecondAscentRate = (firstTenMinutesFinalAltitude - launchDetectedAltitude) / 600;
          //
          //Disable the Altitude Sensor reading
          pauseAltSensorReading = true;
        }

        //This scenario only exists during fast simulations
        if (altDEMA > BMP180_MAX_ALTITUDE && firstTenMinutesFinalAltitude == 0 && pauseAltSensorReading == false) {
          //Disable the Altitude Sensor reading
          pauseAltSensorReading = true;
          firstMinuteFinalAltitude = (int)altitude  + altSimulator;
          firstMinutePerSecondAscentRate = (firstMinuteFinalAltitude - launchDetectedAltitude) / 60;
          firstTenMinutePerSecondAscentRate = firstMinutePerSecondAscentRate;
          altDEMA = (int)altitude + altSimulator;
        }
      }
      //-----------------------------------------------------------------------------------------------------

      //don't revert back to ascending if we have activated the descending state
      if (altDEMA > altLastDEMA && altDEMA - altLastDEMA > MIN_ASCENT_RATE && decendingFlag == false && altDEMA < SLOWPAN_ACTIVE_ALTITUDE) {
        ascendingFlag = true;
        decendingFlag = false;
        floatingFlag = false;
      } else if ((altDEMA < altLastDEMA && altLastDEMA - altDEMA > MIN_DECENT_RATE) || altDEMA >= PEAK_ESTIMATED_ALTITUDE){
        ascendingFlag = false;
        decendingFlag = true;
        floatingFlag = false;
        if (altDEMA < BMP180_MAX_ALTITUDE) {
          //Enable the Altitude Sensor reading
          pauseAltSensorReading = false;
        }
      } else if (altDEMA < LAUNCH_DETECT_ALTITUDE && launchedFlag == true) {
        touchdownFlag = true;
        ascendingFlag = false;
        decendingFlag = false;
      } else if ((ascentRate <= FLOAT_RATE && launchedFlag == true && ascendingFlag == true && altDEMA >= SLOWPAN_ACTIVE_ALTITUDE) || altDEMA >= SLOWPAN_ACTIVE_ALTITUDE) {
        floatingFlag = true;
        ascendingFlag = false;
        decendingFlag = false;
      }   
    }
  }
}




//
// ************************************************
// Control Moment Gyro Actuation Loops
// ************************************************
void Maneuver() {
  //This Sub calls the main sub, passing the appropriate values
  if (cmgActive == false) {
    cmgActive = true;

    if (lastListenSecond != now.second()) {
      listenCount += 1;
      lastListenSecond = now.second();
    }
    if (listenCount > maxListensBeforeHeadingChange) {
      if (valDirection2) {
        targHeading += searchHeadingIncrement;
      } else {
        targHeading -= searchHeadingIncrement;
      }

      //Are we out of range?
      //If so, then reverse the direction!
      if (targHeading >= 360 || targHeading <= 0) {
        valDirection2 = !valDirection2;
        if (targHeading >= 360) {
          targHeading -= searchHeadingIncrement;
        } else {
          targHeading += searchHeadingIncrement;
        }
      }

      //Reset the listen count
      listenCount = 0;
    }



    setCMG3(targHeading, TIMEDOMAIN);
    //setCMG2(targHeading, TIMEDOMAIN);
    cmgActive = false;


    double errHeading = abs(targHeading - bearing);
    
    dwlTotalCount += 1;
    // are we within range?
    if (errHeading > 0 && errHeading < 15) {
      //increment the dwell counter
      dwellCount += 1;



      //have we spent enough time at this heading?  If not, then exit the subroutine
      if (dwellCount < dwellTest) {
        //bleep(1, 1, 2000, -10, 5);
        return;
      }

      dwellTest = 5;

      findTarget = false;
    }


    //decrease the dwell counter...  We missed it this time.
    dwellCount -= 1;
    //check to make sure we haven't gone below the lower range of the dwell counter
    if (dwellCount < 0) {
      dwellCount = 0;
    }

    if (dwlTotalCount > 10) {
      dwlTotalCount = 0;
      dwellTest -= 1;
      if (dwellTest < 1) {
        dwellTest = 1;
      }
      bleep(1, 1, 500, -10, 5, false);
    }

  }
}



void setCMG2(double tgtYAW, uint16_t timeDomain) {
  //This routine is based on similar logic used in quadricopters and other systems
  //where a target is attempting to be reached...
  //This is a Parallel PID

  float lastBearing = bearing;
  float adjTurnBrake;
  ReadCompass2();
  float tempTurnRate=0;

  //http://www.inpharmix.com/jps/PID_Controller_For_Lego_Mindstorms_Robots.html
  //Truth Table - Direction And Result Calculation
  //----------------------------------------------------
  //
  //
  //
  //  ABS(errHeading)  |  Direction  |  tgtYaw < finalHeading  |  > 180  |   Motor Spin  |  Examples: tgt - now
  //-------------------|-------------|-------------------------|---------|---------------|-----------------------------------------
  //      0 - 180      |    CW (1)   |            NO           |    NO   |       CW      |   90 - 30 = 60CW
  //-------------------|-------------|-------------------------|---------|---------------|-----------------------------------------
  //     180 - 360     |   CCW (-1)  |            NO           |   YES   |       CW      |  270 - 30 = 240CW  (shorter as 120CCW)
  //-------------------|-------------|-------------------------|---------|---------------|-----------------------------------------
  //      0 - 180      |   CCW (-1)  |           YES           |    NO   |       CW      |   30 - 90 = 60CCW
  //-------------------|-------------|-------------------------|---------|---------------|-----------------------------------------
  //     180 - 360     |    CW (1)   |           YES           |   YES   |       CW      |  30 - 270 = 240CW  (shorter as 120CCW)
  //-------------------|-------------|-------------------------|---------|---------------|-----------------------------------------

  //#define SERVOMIN  120 // this is the 'minimum' pulse length count (out of 4096)
  //#define SERVOMAX  530 // this is the 'maximum' pulse length count (out of 4096)
  //Kp = (530 - 120)/(180 - 0) = 10
  //Kp = (410)/(180) = 2.27




  double errHeading = abs(tgtYAW - bearing);


  tempTurnRate = (float) (lastBearing - bearing);
  //turnRate = turnRate * .25 + tempTurnRate * .75;
  //turnRate = turnRate * .70 + tempTurnRate * .30;
  turnRate = (turnRate * .50) + (tempTurnRate * .50);

  if (turnRate > 60)
  {
    if (lastBearing > 270 && bearing < 90)
    {
      turnRate = (360 - lastBearing) + bearing;
    }
    else if (lastBearing < 90 && bearing > 270)
    {
      turnRate = (360 - bearing) + lastBearing;
    }
  }
  turnRate = abs(turnRate);




  if (tgtYAW < bearing) {
    valDirection = -1;
  }
  else
  {
    valDirection = 1;
  }

  if (errHeading > 180) {
    //Motor must be spinning CCW
    //This result means the target position is closer if we spin in the opposite direction
    errHeading = 360 - errHeading;
    valDirection = valDirection * -1;
  }


  //if (ShowLEDCount == 0) {
  //  setTargetHeadingLED((errHeading * valDirection));
  //}
  //errTargetHeading = errHeading;


  float u = (PID(errHeading) / 7);
  if (u < 1 && u > .6) {
    u = 1;
  }

  if ((turnRate / 2) > 1) {
    adjTurnBrake = (u * 3);
  }
  else
  {
    adjTurnBrake = (u * 3) * (turnRate);
  }

  if (errHeading < 30) {
    adjTurnBrake = 2;
  }


  //was....  && errHeading < 7
  if (errHeading > 0 && errHeading < 15) {
    adjTurnBrake = 0;
    u = 0;
  }


  //setServo(posLast + (u * valDirection) - adjTurnBrake);
  //setServo4(posLast + (u * valDirection) - adjTurnBrake);
  if ((((int) u) > 1) || (turnRate > 0)) {
    //setServo(posLast + (u * valDirection) - adjTurnBrake);
    //setServo3(posLast + (u * valDirection) - adjTurnBrake);
    //setServo3(posLast + (u * valDirection));
    //setServo4(posLast + (u * valDirection) - adjTurnBrake);
    setServo4(posLast + (u * valDirection));
    //runMotor(MOTORMAX);
  }
}






void setCMG3(double tgtYAW, uint16_t timeDomain) {
  float u;

  
  //This routine is based on similar logic used in quadricopters and other systems
  //where a target is attempting to be reached...
  //This is a Parallel PID


  ReadCompass2();

  valDirection = (GyroTurnRate/abs(GyroTurnRate));
  //Negative is a Clockwise Turn, looking down the Z axis
  //Positive is a Clockwise Turn, looking down the Z axis

  double errHeading = abs(tgtYAW - bearing);
  if (errHeading > 180) {
    //Motor must be spinning CCW
    //This result means the target position is closer if we spin in the opposite direction
    errHeading = 360 - errHeading;
    valDirection = valDirection * -1;
  }
  
  //if (ShowLEDCount == 0) {
    //setTargetHeadingLED((errHeading * valDirection));
  //}
  //errTargetHeading = errHeading;



  //http://www.inpharmix.com/jps/PID_Controller_For_Lego_Mindstorms_Robots.html
  //Truth Table - Direction And Result Calculation
  //----------------------------------------------------
  //
  //
  //
  //  ABS(errHeading)  |  Direction  |  tgtYaw < finalHeading  |  > 180  |   Motor Spin  |  Examples: tgt - now
  //-------------------|-------------|-------------------------|---------|---------------|-----------------------------------------
  //      0 - 180      |    CW (1)   |            NO           |    NO   |       CW      |   90 - 30 = 60CW
  //-------------------|-------------|-------------------------|---------|---------------|-----------------------------------------
  //     180 - 360     |   CCW (-1)  |            NO           |   YES   |       CW      |  270 - 30 = 240CW  (shorter as 120CCW)
  //-------------------|-------------|-------------------------|---------|---------------|-----------------------------------------
  //      0 - 180      |   CCW (-1)  |           YES           |    NO   |       CW      |   30 - 90 = 60CCW
  //-------------------|-------------|-------------------------|---------|---------------|-----------------------------------------
  //     180 - 360     |    CW (1)   |           YES           |   YES   |       CW      |  30 - 270 = 240CW  (shorter as 120CCW)
  //-------------------|-------------|-------------------------|---------|---------------|-----------------------------------------

  //#define SERVOMIN  120 // this is the 'minimum' pulse length count (out of 4096)
  //#define SERVOMAX  530 // this is the 'maximum' pulse length count (out of 4096)
  //Kp = (530 - 120)/(180 - 0) = 10
  //Kp = (410)/(180) = 2.27


  if (abs(GyroTurnRate) > 40) {
    u = PID2(abs(GyroTurnRate)*3);
    //Serial.print("PID_A: ");
    //Serial.print(abs(GyroTurnRate)*3);
  }
  else
  {
    if (errHeading > 45) {
      u = PID2(errHeading);
      //Serial.print("PID_B: ");
      //Serial.print(errHeading);      
    }
    else {
      u = PID2(errHeading+(GyroTurnRate));
      //Serial.print("PID_B: ");
      //Serial.print(errHeading+(GyroTurnRate));      
    }
  }
  //Serial.print(" , ");
  //Serial.print(u);
  //Serial.print(" , ");
  //Serial.print(valDirection);


  if ((abs(GyroTurnRate) > 5) || (errHeading > 10)) {
    setServo4(posLast + (u * valDirection));
    //Serial.print(" *  ");
  }
  //Serial.print(posLast);
  //Serial.println("");

}



//
// ************************************************
// PID Control Loops
// ************************************************
float PID(float valHeading) {
  float result;

  //running sum of the error for the integral calculation
  intgError = (.75 * intgError) + valHeading;
  dervError = valHeading - lastError;

  //-------------------------------------
  float Kp = propGAIN * valHeading;
  float Ki = intGAIN * intgError;
  float Kd = dervGAIN * dervError;

  //float s = timeDomain/1000;
  //result = Kp + (Ki * s) + (Kd * 1/s);
  result = Kp + Ki + Kd;
  lastError = valHeading;

  return result;

}


float PID2(float valError) {
  float result;

  //running sum of the error for the integral calculation
  intgError = (.75 * intgError) + valError;
  dervError = valError - lastError;

  //-------------------------------------
  float Kp = propGAIN * valError;
  float Ki = intGAIN * intgError;
  float Kd = dervGAIN * dervError;

  //float s = timeDomain/1000;
  //result = Kp + (Ki * s) + (Kd * 1/s);
  result = Kp + Ki + Kd;
  lastError = valError;

  return result;

}

