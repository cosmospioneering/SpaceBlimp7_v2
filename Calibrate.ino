

//CMG-01
// posLeft  445
// posRight 85
// posCenter 250

//CMG-23
// posLeft  445
// posRight 85
// posCenter 250

void calibrateServos() {
int pos = 0;    // variable to store the servo position
int intLoop = 0;

  setMotors(MOTORREADY);
  pwm.setPWM(1, 0, posRight);
  //First, we need to find the typical current used during
  //a sweep for each servo
  for (intLoop = 0; intLoop < 3; intLoop += 1) 
  {    
    for (pos = 85; pos < 445; pos += 1) // goes from 0 degrees to 180 degrees
    { // in steps of 1 degree
      delay(15);
      current_mA1 = ina2191.getCurrent_mA();
      Serial.print(pos);
      Serial.print("---PreCurrent 1: ");
      Serial.print(current_mA1);
      pwm.setPWM(1, 0, pos);
      //PWM Ch0 & Ch1
      current_mA1 = ina2191.getCurrent_mA();
      Serial.print("    Post Current 1: ");
      Serial.println(current_mA1);
    }
    for (pos = 445; pos > 85; pos -= 1) // goes from 0 degrees to 180 degrees
    { // in steps of 1 degree
      delay(15);
      current_mA1 = ina2191.getCurrent_mA();
      Serial.print(pos);
      Serial.print("---PreCurrent 1: ");
      Serial.print(current_mA1);
      pwm.setPWM(1, 0, pos);
      //PWM Ch0 & Ch1
      current_mA1 = ina2191.getCurrent_mA();
      Serial.print("    Post Current 1: ");
      Serial.println(current_mA1);
    }
  }
  pwm.setPWM(1, 0, posCenter);

  pwm.setPWM(3, 0, posRight);
  //First, we need to find the typical current used during
  //a sweep for each servo
  for (intLoop = 0; intLoop < 3; intLoop += 1) 
  {    
    //for (pos = posRight-150; pos < posLeft+50; pos += 1) // goes from 0 degrees to 180 degrees
    for (pos = 85; pos < 445; pos += 1) // goes from 0 degrees to 180 degrees
    { // in steps of 1 degree
      delay(15);
      current_mA2 = ina2192.getCurrent_mA();
      Serial.print(pos);
      Serial.print("---PreCurrent 2: ");
      Serial.print(current_mA2);
      pwm.setPWM(3, 0, pos);
      //PWM Ch2 & Ch3??
      current_mA2 = ina2192.getCurrent_mA();
      Serial.print("    Post Current 2: ");
      Serial.println(current_mA2);
    }
    for (pos = 445; pos > 85; pos -= 1) // goes from 0 degrees to 180 degrees
    { // in steps of 1 degree
      delay(15);
      current_mA2 = ina2192.getCurrent_mA();
      Serial.print(pos);
      Serial.print("---PreCurrent 2: ");
      Serial.print(current_mA2);
      pwm.setPWM(3, 0, pos);
      //PWM Ch2 & Ch3??
      current_mA2 = ina2192.getCurrent_mA();
      Serial.print("    Post Current 2: ");
      Serial.println(current_mA2);
    }
  }
  pwm.setPWM(3, 0, posCenter);

 
}



void calibrateESCs() {

      
//MOTORZERO 0
//MOTORREADY 200
//MOTORSTART 290
//MOTORIDLE 300
//MOTORMAX 340
      
      
      setMotors(MOTORZERO);

}









void calcGyroOffset() {
int bearingTemp; int bearingMin = 360; int bearingMax = 0; int bearingMedian = 0;
int gyroMin = 500; int gyroMax = 0; int gyroMedian = 0;


  for (uint16_t steps = 0; steps < 100; steps++)  // steps goes from 0 to 100
  {
    ReadCompass3();getMPUData();
    bearingTemp = (int) bearing;
    //Serial.print(bearingTemp);
    //Serial.print("   ");
    //Serial.println(GyroZ);
    if (bearingTemp < bearingMin) { bearingMin = bearingTemp; }
    if (bearingTemp > bearingMax) { bearingMax = bearingTemp; }
    if (GyroZ < gyroMin) { gyroMin = GyroZ; }
    if (GyroZ > gyroMax) { gyroMax = GyroZ; }
  }
  bearingMedian = bearingMax - bearingMin;
  gyroMedian = gyroMax - gyroMin;

  if (bearingMedian <= 5 & (gyroMax - gyroMedian) > 1) {
    gyro_offset += (gyroMax - gyroMedian);
  }
  /*
  Serial.print("Bearing Data:   ");
  Serial.print(bearingMax);
  Serial.print("   ");
  Serial.println(bearingMin);
  Serial.print("Gyro Data:   ");
  Serial.print(gyroMax);
  Serial.print("   ");
  Serial.println(gyroMin);
  Serial.print("Gyro Offset Data:   ");
  Serial.print(bearingMedian);
  Serial.print("   ");
  Serial.println(gyroMedian);
  */
}






//void initCalibration() {
//  delay(1000);
  //
//  CompassCalibrationCheck();
  //
//}


void CompassCalibrationCheck() {
  //This subroutine starts the motor
  //and checks to see if a turn will yield
  //all four compass point crossings
  double calLastHeading;
  double calHeadingDiff;
  uint16_t countRevolution = 0;


  //Serial.println("Verify Compass Calibration:\t");
  //digitalWrite(13, HIGH);

  //read the initial heading
  ReadCompass2();
  //set the last heading value
  calLastHeading = bearing;

  //center the servo...
  //warning, this won't be graceful if the servo
  //is somewhere else
  pwm.setPWM(1, 0, posCenter);
  //start the motor
  runMotor(MOTORMAX);
  //run the motor up to speed
  runMotor(MOTORMAX);

  //runMotor(MOTORSTART);
  //delay(3000);
  //runMotor(MOTORMAX);
  //slight delay
  delay(300);
  //turn towards the right, or a clockwise spin
  setServo(240);
  //posCenter = 340
  //int posLeft = 530
  //int posRight = 120
  //slight delay
  delay(300);



  //delay(1000);
  for (uint16_t steps = 0; steps < 10; steps++)  // steps goes from 0 to 10
  {
    Serial.println("Sre:\t");
    for (uint16_t calsteps = 0; calsteps < 10; calsteps++)  // calsteps goes from 0 to 10
    {
      // waits 100ms before reading the next position
      digitalWrite(13, LOW);
      delay(100);
      digitalWrite(13, HIGH);
      //read the current heading
      ReadCompass2();

      //Calculate the difference
      calHeadingDiff = bearing - calLastHeading;

      if (calLastHeading < 90 && bearing > 90 && dirPassE == false) {
        dirPassE = true;
        bleep(1, 2, 3500, -500, 5, false);
      }
      if (calLastHeading < 180 && bearing > 180 && dirPassS == false) {
        dirPassS = true;
        bleep(1, 3, 3500, -500, 5, false);
      }
      if (calLastHeading < 270 && bearing > 270 && dirPassW == false) {
        dirPassW = true;
        bleep(1, 4, 3500, -500, 5, false);
      }
      if (calLastHeading < 360 && bearing > 0 && dirPassN == false) {
        dirPassN = true;
        bleep(1, 1, 3500, -500, 5, false);
      }
      if (dirPassE && dirPassS && dirPassW && dirPassN) {
        dirPassE = false;
        dirPassS = false;
        dirPassW = false;
        dirPassN = false;
        countRevolution += 1;
        //bleep(byte numBleeps, byte numTones, word bgnFreq, int stpFreq, byte milDelay)
        bleep(4, 20, 1000, 50, 1, false);
      }
      calLastHeading = bearing;
    }
    //bleep(byte numBleeps, byte numTones, word bgnFreq, int stpFreq, byte milDelay)
    //bleep(1, 20, 3500, -150, 10);
    bleep(1, 10, 3500, -300, 3, false);
  }
  digitalWrite(13, LOW);


  if (countRevolution > 0) {
    bleep(3, 25, 500, 80, 3, false);
  }
  else
  {
    //calibration is needed!!
    bleep(1, 2, 3500, -2000, 10, false);
    pwm.setPWM(1, 0, posCenter);
    delay(500);
    setServo(240);
    delay(500);
    //compass_debug = 0;
    compass_offset_calibration(2);
    SaveParameters();    //Save parameters after the calibration
  }
  dirPassE = false;
  dirPassS = false;
  dirPassW = false;
  dirPassN = false;

}








void runCMGOutputTest() {
int testDelay = 0;
byte index = 0;    // variable to store the index position
int tgtPos = 0;




//This test is designed to test each pass of the CMG
//From center to left, back to center
//And from center to right and back again
//
//The test is looking for evidence that slowing the
//rate the gimbal is updated will affect how much
//torque is output by the CMG


//int16_t storedSpinRate = 0;

  Create_Log_Entry(LogStateChange,0);

  //Center the Gimbals
  resetServos();
  delay(250);   //Minor delay to let things settle

  setMotors(MOTORMAX);
  setMotors(MOTORMAX);

  for (testDelay = -1; testDelay < 5; testDelay++) // goes from 0 degrees to the end of the array
  { // increment in steps of ten

    ReadSensors(LogPower); //Get the date-time!
    //Log Power data right now!
    Create_Log_Entry(LogPower,0);

    bleep(testDelay + 2, 25, 500, 80, 3, 0);


    
 
    if (testDelay < 0) {
      for (index = 0; index < 18; index++) // goes from 0 degrees to the end of the array
      { // increment in steps of ten
          idxGimbalAngle = index;
          getMPUOutputTestVals();
          //Store the SpinRate from BEFORE this application
          storedSpinRate = GyroTurnRate;
      
          //Get the target position
          tgtPos = posCenter + (10 * idxGimbalAngle);
          //Use the Servo's default speed,
          //that means the motion is as fast as the servo can move!
          setServoPos(tgtPos, 0);
          delay(1000);  //Let things settle!
          setServoPos(0, 0);
  
          //Get the resulting spin rate
          getMPUOutputTestVals();

          //Write the data to the EEProm
          //This is a POSITIVE Gyro resulting spin
          Create_Log_Entry(LogCMGPosCal, 0);
          
          //Then reverse apply the spin!
          //
          //Don't change the delay
          //Return the servos to center
          setServoPos(posCenter, 0);
          delay(1000);  //Let things settle!
          setServoPos(0, 0);
 
          getMPUOutputTestVals();
          //Store the SpinRate from BEFORE this Opposite application
          storedSpinRate = GyroTurnRate;
  
          //Get the target position
          tgtPos = posCenter - (10 * idxGimbalAngle);
          setServoPos(tgtPos, 0);
          delay(1000);  //Let things settle!
          setServoPos(0, 0);
          
          //Get the resulting spin rate
          getMPUOutputTestVals();
  
          //Write the data to the EEProm
          //This is a Negati8ve Gyro resulting spin
          Create_Log_Entry(LogCMGNegCal, 0);
  
          //Then reverse apply the spin!
          //
          //Don't change the delay
          //Return the servos to center
          setServoPos(posCenter, 0);
          delay(1000);  //Let things settle!
          setServoPos(0, 0);
      }
    } else {      
      for (index = 0; index < 18 ; index++) // goes from 0 degrees to the end of the array
      { // increment in steps of ten
  
  
          
          idxGimbalAngle = index;
          getMPUOutputTestVals();
          //Store the SpinRate from BEFORE this application
          storedSpinRate = GyroTurnRate;
          
          //Get the target position
          tgtPos = posCenter + (10 * idxGimbalAngle);
          posDelay = testDelay;
          //Advance the servos!
          setServo(tgtPos);
          delay(1000);  //Let things settle!
          setServoPos(0, 0);
  
          
          //Get the resulting spin rate
          getMPUOutputTestVals();
  
          //Write the data to the EEProm
          //This is a POSITIVE Gyro resulting spin
          Create_Log_Entry(LogCMGPosCal, 0);
          
          //Then reverse apply the spin!
          //
          //Don't change the delay
          //Return the servos to center
          setServo(posCenter);
          delay(1000);  //Let things settle!
          setServoPos(0, 0);
  
          
          getMPUOutputTestVals();
          //Store the SpinRate from BEFORE this Opposite application
          storedSpinRate = GyroTurnRate;
  
          //Get the target position
          tgtPos = posCenter - (10 * idxGimbalAngle);
          posDelay = testDelay;
          //Advance the servos!
          setServo(tgtPos);
          delay(1000);  //Let things settle!
          setServoPos(0, 0);
          
          //Get the resulting spin rate
          getMPUOutputTestVals();
  
          //Write the data to the EEProm
          //This is a Negati8ve Gyro resulting spin
          Create_Log_Entry(LogCMGNegCal, 0);
  
          //Then reverse apply the spin!
          //
          //Don't change the delay
          //Return the servos to center
          setServo(posCenter);
          delay(1000);  //Let things settle!
          setServoPos(0, 0);
  
      }
    }
  }

  
}



void runCMGOutputTest_2() {
int testDelay = 0;
byte index = 0;    // variable to store the index position
int tgtPos = 0;

//This test is designed to test a single pass of the CMG
//From center to left, back to center
//
//The test is looking for evidence that slowing the
//rate the gimbal is updated will affect how much
//torque is output by the CMG when incrementing the delay from zero to 5
//for the RETURN from the fully advanced position back to center


  Create_Log_Entry(LogStateChange,0);

  //Center the Gimbals
  resetServos();
  delay(250);   //Minor delay to let things settle

  setMotors(MOTORMAX);
  setMotors(MOTORMAX);

  for (testDelay = 0; testDelay < 5; testDelay++) // goes from 0 degrees to the end of the array
  { // increment in steps of ten

    ReadSensors(LogPower); //Get the date-time!
    //Log Power data right now!
    Create_Log_Entry(LogPower,0);

    bleep(testDelay + 2, 25, 500, 80, 3, 0);

 
      for (index = 0; index < 18; index++) // goes from 0 degrees to the end of the array
      { // increment in steps of ten
          idxGimbalAngle = index;
          getMPUOutputTestVals();
          //Store the SpinRate from BEFORE this application
          storedSpinRate = GyroTurnRate;
      
          //Get the target position
          tgtPos = posCenter + (10 * idxGimbalAngle);
          //Use the Servo's default speed,
          posDelay = testDelay * 5;
          //Advance the servos!
          setServoPos(tgtPos, 0);
          delay(1000);  //Let things settle!
          setServoPos(0, 0);
  
          //Get the resulting spin rate
          getMPUOutputTestVals();

          //Write the data to the EEProm
          //This is a POSITIVE Gyro resulting spin
          Create_Log_Entry(LogCMGPosCal, 0);
          
          //Get the resulting spin rate
          getMPUOutputTestVals();
          //Store the SpinRate from BEFORE this application
          storedSpinRate = GyroTurnRate;

          //Then reverse apply the spin!
          //
          //Return the servos to center, with no delay!
          setServo(posCenter);
          delay(1000);  //Let things settle!
          setServoPos(0, 0);
 
          getMPUOutputTestVals();
         
          //Write the data to the EEProm
          //This is a Negative Gyro resulting spin
          Create_Log_Entry(LogCMGNegCal, 0);
      }

    }
  
}






void runCMGOutputTest_3() {
int testDelay = 10;
byte index = 0;    // variable to store the index position
int tgtPos = 0;




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

  Create_Log_Entry(LogStateChange,0);

  //Center the Gimbals
  resetServos();
  delay(250);   //Minor delay to let things settle
  //Use the Test Delay for Servo speed,
  posDelay = testDelay;


  for (index = 0; index < 15; index++) // goes from 0 degrees to the end of the array
  { 
    //   1>  Store the spin rate
      idxGimbalAngle = index;
      getMPUOutputTestVals();
      //Store the SpinRate from BEFORE this application
      storedSpinRate = GyroTurnRate;

      
      
    //   2>  While the Gyro is centered, decrease motor speed to zero, Log the spin rate
      setMotors(MOTORREADY);
      delay(800);  //Let things settle!
      MotorRunning = false;
      getMPUOutputTestVals();
      //Write the data to the EEProm
      //This is a POSITIVE Gyro resulting spin
      Create_Log_Entry(LogCMGPosCal, 0);


    
    //   3>  Gimbal the Gyro From center to left at a delayed speed, Store the spin rate
      //Get the target position
      tgtPos = posRight;
      //Advance the servos!
      posDelay = 5;
      setServo(tgtPos);
      //delay(10 * index);  //Let things settle!
      setServoPos(0, 0);
      getMPUOutputTestVals();
      //Store the SpinRate from BEFORE this application
      storedSpinRate = GyroTurnRate;


    
    //   4>  Start the motor, Log the spin rate
      setMotors(MOTORMAX);
      setMotors(MOTORMAX);
      //delay(100 * index);  //Delay the start!
      delay(1500);  //Let things settle!
      getMPUOutputTestVals();
      //Write the data to the EEProm
      //This is a Negative Gyro resulting spin
      Create_Log_Entry(LogCMGNegCal, 0);
  
    
    //   5>  Store the spin rate, Gimbal the Gyro back to center at a delayed speed, Log the spin rate
      //setMotors(MOTORREADY);
      //delay(100);  //Let things settle!
      //getMPUOutputTestVals();
      //Store the SpinRate from BEFORE this application
      storedSpinRate = GyroTurnRate;
      //Get the target position
      tgtPos = posCenter;
      posDelay = index;
      //Advance the servos!
      setServo(tgtPos);
      delay(500);  //Let things settle!
      setServoPos(0, 0);
      getMPUOutputTestVals();
      //Write the data to the EEProm
      //This is a POSITIVE Gyro resulting spin
      Create_Log_Entry(LogCMGPosCal, 0);
    
    //   6>  Loop through this process ten times to see if there is an increase in torque in a specific direction
  }

  setMotors(MOTORREADY);

}





void runCMGOutputTest_3a() {
int testDelay = 10;
int index = 0;    // variable to store the index position
int index2 = 0;    // variable to store the index position
int tgtPos = 0;




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

  Create_Log_Entry(LogStateChange,0);

  //Center the Gimbals
  resetServos();
  delay(250);   //Minor delay to let things settle
  //Use the Test Delay for Servo speed,
  posDelay = testDelay;


  for (index2 = 3; index2 < 7; index2++) // goes from 0 degrees to the end of the array
  { 
    bleep(index2 - 2, 25, 500, 80, 3, 0);
    //for (index = 7; index < 20; index++) // goes from 0 degrees to the end of the array
    for (index = 13; index < 16; index++) // goes from 0 degrees to the end of the array
    {       
      //   3>  Gimbal the Gyro From center to left at a delayed speed, Store the spin rate
        //Get the target position
        tgtPos = posRight;
        //Advance the servos!
        posDelay = 5;
        setServo(tgtPos);
        //delay(10 * index);  //Let things settle!
        setServoPos(0, 0);
  
  
      
      //   4>  Start the motor, Log the spin rate
        setMotors(MOTORMAX);
        setMotors(MOTORMAX);
        delay(100 * index);  //Prolong the start & run time!
        //delay(1500);  //Let things settle!
    
    
      //   5>  Store the spin rate, Gimbal the Gyro back to center at a delayed speed, Log the spin rate
        setMotors(MOTORREADY);
        MotorRunning = false;
        delay(100 * index2);  //Delay the shutdown
        //Get the target position
        tgtPos = posCenter;
        posDelay = 5;
        //posDelay = index;
        //Advance the servos!
        setServo(tgtPos);
        delay(600);  //Let things settle!
        setServoPos(0, 0);   
    }
    
    //   6>  Loop through this process ten times to see if there is an increase in torque in a specific direction
  }

  setMotors(MOTORREADY);

}




void runCMGOutputTest_3b() {
int testDelay = 10;
int index = 0;    // variable to store the index position
int index2 = 0;    // variable to store the index position
int tgtPos = 0;
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

  Create_Log_Entry(LogStateChange,0);

  //Center the Gimbals
  resetServos();
  delay(250);   //Minor delay to let things settle
  //Use the Test Delay for Servo speed,
  posDelay = testDelay;


    for (index = 0; index < 50; index++) // goes from 0 degrees to the end of the array
    {       
      //   3>  Gimbal the Gyro From center to left at a delayed speed, Store the spin rate
        //Get the target position
        tgtPos = posRight;
        //Advance the servos!
        posDelay = 5;
        setServo(tgtPos);
        //delay(10 * index);  //Let things settle!
        setServoPos(0, 0);
  
  
      
      //   4>  Start the motor, Log the spin rate
        setMotors(MOTORMAX);
        setMotors(MOTORMAX);
        delay(100 * delay1);  //Prolong the start & run time!
    
    
      //   5>  Store the spin rate, Gimbal the Gyro back to center at a delayed speed, Log the spin rate
        setMotors(MOTORREADY);
        MotorRunning = false;
        delay(100 * delay2);  //Delay the shutdown
        //Get the target position
        tgtPos = posCenter;
        posDelay = 5;
        setServo(tgtPos);
        delay(600);  //Let things settle!
        setServoPos(0, 0);   
    }

  setMotors(MOTORREADY);

}


void runCMGOutputTest_4() {
int testDelay = 10;
byte index = 0;    // variable to store the index position
int tgtPos = 0;




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

  Create_Log_Entry(LogStateChange,0);

  //Center the Gimbals
  resetServos();
  delay(250);   //Minor delay to let things settle
  //Use the Test Delay for Servo speed,
  posDelay = testDelay;


  for (index = 0; index < 15; index++) // goes from 0 degrees to the end of the array
  {       

    //   2>  While the Gyro is centered, decrease motor speed to zero, Log the spin rate
      MotorRunning = true;
      setMotors(MOTORREADY);
      MotorRunning = false;
      //Get the target position
      tgtPos = posLeft;
      posDelay = 2;
      setServo(tgtPos);    
      //setServoPos(tgtPos, 0);
      delay(800);  //Let things settle!

    
      posDelay = 5;
    //   3>  Gimbal the Gyro From center to left at a delayed speed, Store the spin rate
      //Get the target position
      tgtPos = posRight;
      //Advance the servos!
      setServo(tgtPos);
      //delay(10 * index);  //Let things settle!
      setServoPos(0, 0);


    
    //   4>  Start the motor, Log the spin rate
      setMotors(MOTORMAX);
      setMotors(MOTORMAX);
      //delay(100 * index);  //Delay the start!
      delay(700);  //Let things settle!
  
    
    //   5>  Store the spin rate, Gimbal the Gyro back to center at a delayed speed, Log the spin rate
      //setMotors(MOTORREADY);
      //delay(100);  //Let things settle!
      //getMPUOutputTestVals();
      //Store the SpinRate from BEFORE this application
      storedSpinRate = GyroTurnRate;
      //Get the target position
      tgtPos = posCenter;
      posDelay = index;
      //Advance the servos!
      setServo(tgtPos);
    //   6>  Loop through this process ten times to see if there is an increase in torque in a specific direction
  }

  setMotors(MOTORREADY);

}









void getMPUOutputTestVals() {
  //Get the initial Spin Rate
  getMPUData();
  GyroTurnRate = GyroZ;
  delay(100);
  getMPUData();
  delay(100);
  getMPUData();
  delay(100);
  getMPUData();
  delay(100);
  getMPUData();
  delay(100);
}


