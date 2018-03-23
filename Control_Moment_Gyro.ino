//--------------------------------------------------------------------------------------------------------------------------
//
//
//   This section contains Control Moment Gyro & Attitude Control subroutines
//
//
//--------------------------------------------------------------------------------------------------------------------------
//
// ************************************************
// Initialize the ESCs (and thus, the motors)
// ************************************************
//
//
/*
void coldStartESCs_v1() {

  resetServos();  //center the servo...
  
//MOTORZERO 0
//MOTORREADY 200
//MOTORSTART 290
//MOTORIDLE 300
//MOTORMAX 340

//Old settings
  //MOTORZERO = 0
  //MOTORREADY = 200
  //MOTORSTART = 250
  //MOTORIDLE = 2    80
  //MOTORTOP = 320
  //MOTORMAX = 375

  bleep(1, 25, 500, 80, 3, 0);
  //Use this sequence to reset the Min/Max settings on the Mx-3A ESC!!
  setMotors(MOTORZERO);  //Set the PWM to 0%, instructing the ESC that we are at the ZERO position
  delay(7000);  //These timings are critical here...  Wait seven seconds
  bleep(1, 25, 500, 80, 3, 0);
  setMotors(MOTORMAX);  //Set the PWM to 100%
  delay(1000);  //These timings are critical here...
  bleep(1, 25, 500, 80, 3, 0);
  setMotors(MOTORSTART);  
  delay(1000);  //These timings are critical here...
  bleep(1, 25, 500, 80, 3, 0);
  setMotors(MOTORREADY); 
  delay(7000);  //These timings are critical here...
}
*/
void coldStartESCs() {

  resetServos();  //center the servo...
  
//MOTORZERO 0
//MOTORREADY 200
//MOTORSTART 290
//MOTORIDLE 300
//MOTORMAX 340

//Old settings
  //MOTORZERO = 0
  //MOTORREADY = 200
  //MOTORSTART = 250
  //MOTORIDLE = 2    80
  //MOTORTOP = 320
  //MOTORMAX = 375

  //bleep(1, 25, 500, 80, 3, 0);
  //Use this sequence to reset the Min/Max settings on the Mx-3A ESC!!
  setMotors(MOTORZERO);  //Set the PWM to 0%, instructing the ESC that we are at the ZERO position
  delay(1000);  //These timings are critical here...  Wait seven seconds
  //delay(7000);  //These timings are critical here...  Wait seven seconds
  //bleep(1, 25, 500, 80, 3, 0);
  setMotors(MOTORMAX);  //Set the PWM to 100%
  delay(1000);  //These timings are critical here...
  //bleep(1, 25, 500, 80, 3, 0);
  setMotors(MOTORSTART);  
  delay(1000);  //These timings are critical here...
  //bleep(1, 25, 500, 80, 3, 0);
  setMotors(MOTORREADY); 
  //delay(7000);  //These timings are critical here...
  ReadSensors(0);
  nextMotorKickSecond = now.second() + 50;
  if (nextMotorKickSecond > 60) { nextMotorKickSecond += -60; }
}



void runMotor(uint16_t setMOTOR) {

  if (MotorRunning == false)
  {
    //wdt_reset(); // confirm to watchdog timer that all is well
    //MotorRunning = true;
    //setMotors(MOTORZERO);
    //delay(3000);
    //wdt_reset(); // confirm to watchdog timer that all is well
    //setMotors(MOTORREADY);
    //delay(2000);
    //wdt_reset(); // confirm to watchdog timer that all is well
    setMotors(MOTORSTART);
    delay(2000);
    restartMotorCount++;
    //wdt_reset(); // confirm to watchdog timer that all is well
  }
  else
  {
    setMotors(setMOTOR);
  }
}

void warmStartMotor() {
  
}

void setMotors(uint16_t setMOTOR) {
    pwm.setPWM(0, 0, setMOTOR);
    pwm.setPWM(2, 0, setMOTOR);
    LastMOTOR = setMOTOR;  
}



void kickESCs() {
  //This subroutine is to prevent the ESCs from disarming
  //And thus generating a power draining beep!
  //Serial.print("KickESCs:  ");
  //Serial.print(nextMotorKickSecond);
  //Serial.print("   ");
  //Serial.print(now.second());
  if (nextMotorKickSecond == now.second() && LastMOTOR <= MOTORREADY) {
    //Serial.print("   yes");
    pwm.setPWM(0, 0, MOTORMAX);
    pwm.setPWM(2, 0, MOTORMAX);
    delay(20);
    setMotors(LastMOTOR);
    nextMotorKickSecond = now.second() + 50;
    if (nextMotorKickSecond >= 60) { nextMotorKickSecond += -60; }
    //setSimpleLED(HIGH,0);
    //delay(100);
    //setSimpleLED(LOW,0);
  }
  //Serial.println(" ");
}




//
// ************************************************
// Set the Servo Positions
// ************************************************

void resetServos() {
  //center the servo...
  //warning, this won't be graceful if the servo
  //is somewhere else
  setServoPos(posCenter, 0);
  delay(1000);
  //Set the Servos to Zero
  setServoPos(0, 0);
}

void setServoPos(int posSet, int myDelay) {
  if (posSet > 0) { 
    pwm.setPWM(1, 0, posSet + CMG1Offset);
    pwm.setPWM(3, 0, posSet + CMG3Offset);
  } else {
    pwm.setPWM(1, 0, posSet);
    pwm.setPWM(3, 0, posSet);
  }
  //Don't record the zero setting!
  //We MUST remember the last VALID setting...
  if (posSet > 0) { posLast = posSet; }
  delay(myDelay);
  //Serial.print("Servo: ");
  //Serial.println(posLast);
}



/*
void setServo(int sumPosition) {
  boolean reCenter = false;
  int pos = 0;    // variable to store the servo position


  if (sumPosition < posRight || sumPosition > posLeft) {
    sumPosition = posCenter;
    runMotor(MOTORSTART);
    bleep(1, 2, 3500, -2000, 10, false);
    reCenter = true;
  }

  if (sumPosition < posLast)
  {
    for (pos = posLast; pos > sumPosition; pos -= 1) // goes from 0 degrees to 180 degrees
    { // in steps of 1 degree
      setServoPos(pos, posDelay);  // waits 15ms for the servo to reach the position
    }
  }
  else
  {
    for (pos = posLast; pos < sumPosition; pos += 1) // goes from 0 degrees to 180 degrees
    { // in steps of 1 degree
      setServoPos(pos, posDelay);  // waits 15ms for the servo to reach the position
    }
  }
  //posLast = sumPosition;
  if (reCenter) {
    runMotor(MOTORMAX);
  }

}
*/


void setServo4(int sumPosition) {
  boolean reCenter = false;
  int pos = 0;    // variable to store the servo position

  if (posLast <= posRight || posLast >= posLeft) {
    sumPosition = posCenter;
    bleep(1, 2, 3500, -2000, 10, false);
    runMotor(MOTORSTART);
    delay(500);
    reCenter = true;
    posDelay = 3;
  }
  else
  {
    runMotor(MOTORMAX);
    posDelay = 10;
  }


  if (reCenter) {
    runMotor(MOTORIDLE);
    delay(500);
    if (sumPosition < posLast)
    {
      resetServoCount++;
      for (pos = posLast; pos > sumPosition; pos -= 1) // goes from 0 degrees to 180 degrees
      { // in steps of 1 degree
        setServoPos(pos, posDelay);
      }
    }
    else
    {
      resetServoCount++;
      for (pos = posLast; pos < sumPosition; pos += 1) // goes from 0 degrees to 180 degrees
      { // in steps of 1 degree
        setServoPos(pos, posDelay);
      }
    }
  }
  else
  {   
    if (sumPosition >= posRight && sumPosition <= posLeft) {
      if (sumPosition < posLast)
      {
        pos = posLast - ((posLast - sumPosition)/4);
      }
      else
      {
        pos = posLast + ((sumPosition - posLast)/4);
      }
    }
    
    setServoPos(pos, posDelay);
  }

}




/*
void fetchCMGDriveParameters(int SpinAmount) {
byte index = 0;    // variable to store the index position

  storedSpinRate = SpinAmount;
  //First, determine the appropriate direction to compare against
  if (storedSpinRate > 0) {
    //This is a CCW spin
    for (index = 0; index < sizeof(PosSpinRates); index++) // goes from 0 degrees to the end of the array
    { // increment in steps of ten
      if (SpinAmount >= PosSpinRates[index]) {
        idxGimbalAngle = index;
        break;
      }
    }
    dlyGimbalAdv = PosGimbalAdvDelay[idxGimbalAngle];
    dlyGimbalRtn = PosGimbalRtrnDelay[idxGimbalAngle];
  } else if (storedSpinRate < 0) {
    //This is a CW spin!
    for (index = 0; index < sizeof(NegSpinRates); index++) // goes from 0 degrees to the end of the array
    { // increment in steps of ten
      if (SpinAmount <= NegSpinRates[index]) {
        idxGimbalAngle = index;
        break;
      }
    }
    dlyGimbalAdv = NegGimbalAdvDelay[idxGimbalAngle];
    dlyGimbalRtn = NegGimbalRtrnDelay[idxGimbalAngle];
  } else {
    //set the targets to center!
    idxGimbalAngle = -1;
  }
}
*/


void applyCMGTorqueDrive(int dirTorque) {
int testDelay = 5;
int delay1 = 11;  //Prolong the start & run time!
int delay2 = 4;   //Delay between shutdown & gimbal start



//This test is designed to test a single pass of the CMG
//by executing the following steps
//   1>  Store the spin rate
//   2>  While the Gyro is centered, decrease motor speed to zero, Log the spin rate
//   3>  Gimbal the Gyro From center to left at a delayed speed, Store the spin rate
//   4>  Start the motor, Log the spin rate
//   5>  Store the spin rate, Gimbal the Gyro back to center at a delayed speed, Log the spin rate
//   6>  Loop through this process ten times to see if there is an increase in torque in a specific direction
//
//
//The test is looking for evidence that starting the motor off-axis
//will impart spin in a given direction
//and regardless of the amount of torque produced by the return to center
//...
//will produce a positive output


  getMPUOutputTestVals();
  //Store the SpinRate from BEFORE this application
  storedSpinRate = GyroTurnRate;



  if (LastMOTOR > MOTORREADY) {
    setMotors(MOTORREADY);
    delay(800);  //Let things settle!
  }
  //Center the Gimbals
  resetServos();
  delay(250);   //Minor delay to let things settle
  //Use the Test Delay for Servo speed,
  posDelay = testDelay;


      //   3>  Gimbal the Gyro From center to left at a delayed speed, Store the spin rate
        //Advance the servos!
        setServo(dirTorque);
        //delay(10 * index);  //Let things settle!
        setServoPos(0, 0);  
      
      //   4>  Start the motor, Log the spin rate
        setMotors(MOTORMAX);
        delay(100 * delay1);  //Prolong the start & run time!

        //Added in a incrementally longer delay,
        //based on how many stabilization attempts we have made to this point!
        delay(1000 * stabilizedCount);
        getMPUData();

    
      //   5>  Store the spin rate, Gimbal the Gyro back to center at a delayed speed, Log the spin rate
        setMotors(MOTORREADY);
        delay(100 * delay2);  //Delay the shutdown
        setServo(posCenter);
        delay(600);  //Let things settle!
        setServoPos(0, 0);   

  //Get the resulting spin rate
  getMPUOutputTestVals();
  //Write the data to the EEProm
  //This is a POSITIVE Gyro resulting spin
  Create_Log_Entry(LogCMGPosCal, 0);


}



void setServo(int sumPosition) {
  int pos = 0;    // variable to store the servo position

  if (sumPosition < posLast)
  {
    for (pos = posLast; pos > sumPosition; pos -= 1) // goes from 0 degrees to 180 degrees
    { // in steps of 1 degree
      setServoPos(pos, posDelay);  // waits 15ms for the servo to reach the position
    }
  }
  else
  {
    for (pos = posLast; pos < sumPosition; pos += 1) // goes from 0 degrees to 180 degrees
    { // in steps of 1 degree
      setServoPos(pos, posDelay);  // waits 15ms for the servo to reach the position
    }
  }

}


