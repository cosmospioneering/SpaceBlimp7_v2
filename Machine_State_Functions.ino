




//This is what is required for the State Machine Support
//--------------------------------------------------------------------------------------------------------------------------
//    Arduino Setup Section
//--------------------------------------------------------------------------------------------------------------------------
//*********************************
//            Setup Arduino Pins
//*********************************
//void setVariables() {
//  uint32_t currentFrequency;
//}

void setPins() {
  analogReference(INTERNAL);

  //This pin is used for system activity feedback to the user
  pinMode(simpleLEDPin, OUTPUT);
  digitalWrite(simpleLEDPin, LOW);

}














//--------------------------------------------------------------------------------------------------------------------------
//
//
//   This section contains State Machine Support
//
//
//--------------------------------------------------------------------------------------------------------------------------
//-------------------------------------------------------
//    State Machine Setup
//-------------------------------------------------------
//******************************
//            Start-Up State
//******************************
//
//Mission Logging - Events - Active
//Feedback - Audio, Simple LED, Serial Debug - Active
//
void start_up_on_enter()
{
  //Beep!
  //Before we do anything else, let the user know we are doing something!
  bleep(3, 25, 500, 80, 3, 0);

  //
  //Light the LED!
  setSimpleLED(HIGH,0);
  delay(1000);
  LEDSetup = B00000000;
  CycleLEDs();

  //
  //Create a start-up log entry
  ReadSensors(0); //Get the date-time!
  //
  Create_Log_Entry(LogStateChange,0);



  calcGyroOffset();
  

  //Initialize the CMG ESCs
  //Center the Servos
  //
  //Transition to Idle State
  setMachineState(IDLE_EVENT, true);

}

void start_up_on_exit()
{
  //Turn off the LED!
  setSimpleLED(LOW,0);

  //
  //Zero the ESCs
  //runMotor(MOTORREADY);
  setMotors(MOTORREADY);
  //Zero the Servos
  resetServos();
}




//******************************
//            Idle State
//******************************
//
//Mission Logging - Events, Sensors - Active
//Sensing - Power, Environmental - Active
//Feedback - Simple LED - Active
//
void idle_on_enter()
{
  //Activate the LED!
  setSimpleLED(HIGH,0);
  //
  ReadSensors(LogPowerAndSensor); //Get the date-time!
  //
  //Create an idle log entry 
  Create_Log_Entry(LogStateChange,0);
  //Log Power data right now!
  Create_Log_Entry(LogPower,0);

}

void idle_on_state()
{

  //Pulse the LED!
  setSimpleLED(HIGH,300);

  bleep(5, 20, 1000, 50, 1, 0);  //negative buzz
  bleep(1, 70, 800, 10, 5, 0);  //slow alert sound

  ReadSensors(LogPowerAndSensor);
  kickESCs();
  //calcFloatStates();

  //
  //Log sensor data every one minute
  Create_Log_Entry(LogSensor,1);
  //Log Power data every ten minutes
  Create_Log_Entry(LogPower,10);

  //
  //Check Altimeter for ascending values
  //transition to Ascending State when launched is true
  if (launchedFlag == true) { setMachineState(ASCEND_EVENT, true); }

}

void idle_on_exit()
{
  //Turn off the LED!
  //
  setSimpleLED(LOW,0);
  //Serial.println("Exit Idle");

}




//******************************
//            Ascend State
//******************************
//
//Mission Logging - Events, Sensors, Guidance - Active
//Sensing - Power, Environmental, Guidance - Active
//Feedback - Advanced LED - Active
//
void ascend_on_enter()
{

  bleep(1, 30, 2500, -66, 3, 0);  //slow alert sound
  LEDSetup = B00000001;
  ReadSensors(LogSensor);
  //
  //Log sensor data
  Create_Log_Entry(LogSensor,0);
  //Create an idle log entry 
  Create_Log_Entry(LogStateChange,0);
  //
  setMotors(MOTORREADY);
  //Zero the Servos
  resetServos();

}

void ascend_on_state()
{

  //Pulse the LED!
  setSimpleLED(HIGH,250);
  ReadSensors(LogGuidanceAndPowerAndSensor);
  kickESCs();
  //calcFloatStates();
  //Serial.print(bearing);
  //Serial.print("     ");
  //Serial.println(getMPU_GyZ());

  //
  //Log sensor data every one minute
  //Log guidance data every one minute
  Create_Log_Entry(LogPowerAndSensor,1);

  //
  //Check Altimeter for unexpected descending values
  //Transition to Slow-Pan State when ascending is false
  //This needs to take priority over stabilization!
  if (decendingFlag == true) { setMachineState(UNEXPECTED_DECEND_EVENT, true); }
  //
  //Check Gyro for excessive spin
  //Transition to Stabilize when spin exceeds tolerated value
  //This should have priority over floating/slow-pan activities
  if (activeStabilize == true) { setMachineState(STABILIZING_EVENT, true); }
  //
  //Check Altimeter for slower ascending values
  //Transition to Slow-Pan State when ascending is false
  if (floatingFlag == true) { setMachineState(SLOWPAN_EVENT, false); }
}

void ascend_on_exit()
{
  //Turn off the LED!
  //
  setSimpleLED(LOW,0);
  //Serial.println("Exit Ascend");
}



//******************************
//            Stabilize State
//******************************
//
//Mission Logging - Events, Sensors, Guidance - Active
//Sensing - Power, Environmental, Guidance - Active
//Navigation - Servos, Gyros
//Feedback - Advanced LED - Active??
//
void stabilize_on_enter()
{
  
  bleep(1, 25, 500, 80, 3, 0);
  ReadSensors(LogGuidanceAndSensor);

  //
  //Log sensor data
  Create_Log_Entry(LogSensor,0);
  //Create a stabilize log entry
  Create_Log_Entry(LogStateChange,0);

}

void stabilize_on_state()
{

  ReadSensors(LogGuidanceAndPowerAndSensor);
  kickESCs();
  //
  //
  //Log sensor data every one minute
  //Log guidance data every one minute
  Create_Log_Entry(LogPowerAndSensor,1);
  //

  //if (abs(GyroTurnRate) > TARGET_ACCEPTIBLE_SPIN ) {
  if (activeStabilize == true) {
    //if (GyroTurnRate > 0) {
    //  applyCMGTorqueDrive(posLeft);
    //} else if (GyroTurnRate < 0) {
    //  applyCMGTorqueDrive(posRight);
    //}

    if ( preTorqueDriveTurnRate !=0 && postTorqueDriveTurnRate !=0 && stabilizedCount > 0) {
      stabilizedCount = stabilizedCount + 5;
      if (GyroTurnRate > 0) {
        if ( preTorqueDriveTurnRate > postTorqueDriveTurnRate && postTorqueDriveTurnRate <=0 ) {
          bleep(1, 25, 500, 80, 3, 0);
          applyCMGTorqueDrive(posRight);
        }
      } else if (GyroTurnRate < 0) {
        if ( preTorqueDriveTurnRate < postTorqueDriveTurnRate && postTorqueDriveTurnRate >=0 ) {
          bleep(1, 25, 500, 80, 3, 0);
          applyCMGTorqueDrive(posLeft);
        }
      }
      stabilizedCount = stabilizedCount - 5;
    }
    preTorqueDriveTurnRate = GyroTurnRate;
    if (GyroTurnRate > 0) {
      applyCMGTorqueDrive(posLeft);
    } else if (GyroTurnRate < 0) {
      applyCMGTorqueDrive(posRight);
    }
    postTorqueDriveTurnRate = GyroTurnRate;
  }

  //
  //Check Altimeter for descending values
  //Transition to Descending State when ascending is false
  //This needs to take priority over stabilization!
  if (decendingFlag == true) { setMachineState(STABILIZED_DECEND_EVENT, true); }
  //
  //Check Gyro for controlled spin
  //Transition to Ascend when spin is under control
  //and ascending is still active
  //This should have priority over floating/slow-pan activities
  if (activeStabilize == false) { setMachineState(STABLE_ASCEND_EVENT, false); }
  //
  //Check Altimeter for slower ascending values
  //Transition to Slow-Pan State when ascending is false
  if (floatingFlag == true && activeStabilize == false) { setMachineState(STABILIZED_SLOWPAN_EVENT, false); }
}

//******************************
//            Slow-Pan State
//******************************
//
//Mission Logging - Events, Sensors, Guidance - Active
//Sensing - Power, Environmental, Guidance - Active
//Navigation - Servos, Gyros
//
void slowpan_on_enter()
{
  bleep(6, 30, 2500, -66, 3, 0);  //slow alert sound
  ReadSensors(LogGuidanceAndSensor);
  //
  //Log sensor data
  Create_Log_Entry(LogSensor,0);
  //
  //Create a slowpan log entry
  Create_Log_Entry(LogStateChange,0);
  //
  //Activate the CMG ESCs, if they aren't alreay active!
  if ( LastMOTOR <= MOTORREADY) {
    //run the motor up to speed
    runMotor(MOTORMAX);  
  }

}

void slowpan_on_state()
{
 
  //Pulse the LED!
  setSimpleLED(HIGH,1000);

  ReadSensors(LogGuidanceAndPowerAndSensor);
  kickESCs();
  //
  //
  //Log sensor data every one minute
  //Log guidance data every one minute
  Create_Log_Entry(LogPowerAndSensor,1);

  //if (abs(GyroTurnRate) > TARGET_ACCEPTIBLE_SPIN ) { Maneuver(); }
  Maneuver();
  
  //
  //Check Altimeter for descending values
  //Transition to Descending State when ascending is false
  //This needs to take priority over stabilization!
  if (decendingFlag == true) { setMachineState(SLOWPAN_DECEND_EVENT, true); }
  //
  //Check Gyro for uncontrolled spin
  //Transition to Stabilize to regain control
  if (activeStabilize == true) { setMachineState(SLOWPAN_STABILIZE_EVENT, true); }
}

void slowpan_on_exit()
{
  //Turn off the LED!
  //
  setSimpleLED(LOW,0);
  //Serial.println("Exit SlowPan");
}



//******************************
//            Descend State
//******************************
//
//Mission Logging - Events, Sensors, Guidance - Active
//Sensing - Power, Environmental, Guidance - Active
//
void descend_on_enter()
{


  bleep(5, 20, 1000, 50, 1, 0);  //negative buzz
  bleep(3, 70, 800, 10, 5, 0);  //slow alert sound
  ReadSensors(LogSensor);
  //
  //Log sensor data
  Create_Log_Entry(LogSensor,0);
  //Create a descend log entry
  Create_Log_Entry(LogStateChange,0);
  //
  //Deactivate the CMGs
  //Idle the ESCs
  //Center the servos
  //Zero the ESCs
  //Zero the Servos
  //
  setMotors(MOTORREADY);
  //Set the Servos to Zero
  setServoPos(0, 0);
  //

}

void descend_on_state()
{

  //Pulse the LED!
  setSimpleLED(HIGH,100);

  ReadSensors(LogGuidanceAndPowerAndSensor);
  kickESCs();
  //
  //Log sensor data every one minute
  //Log guidance data every one minute
  Create_Log_Entry(LogPowerAndSensor,1);
  //


  //Check Altimeter for descending values
  //Transition to TouchDown State when altitude stabilizes
  if (touchdownFlag == true) { setMachineState(TOUCHDOWN_EVENT, true); }
}




//******************************
//            Touch-Down State
//******************************
//
//Mission Logging - Sensors - Active
//Sensing - Power, Environmental - Active
//Feedback - Audio - Active
//
void touchdown_on_enter()
{
  //
  //Create a touchdown log entry
  Create_Log_Entry(LogStateChange,0);

  //Turn off the LED!
  //
  setSimpleLED(LOW,0);

}

void touchdown_on_state()
{
  //Audio Chirp (Every thirty seconds)
  //bleep(6, 25, 500, 80, 3, 30000);  //alert sound
  bleep(5, 70, 1500, -10, 4, 30000);  //slow alert sound

  //bleep(byte numBleeps, byte numTones, word bgnFreq, int stpFreq, byte milDelay, long repeatDelay)
  //
  ReadSensors(LogPower);
  kickESCs();
  //
  //Read the Environment Sensors (Temp)
  //
  //Log sensor data every one minute
  Create_Log_Entry(LogSensor,1);
  //

}




//******************************
//            Fail-Safe State
//******************************
//
//Mission Logging - Sensors - Active
//Sensing - Power, Environmental - Active
//Feedback - Audio - Active
//
void failsafe_on_enter()
{
  //
  //Deactivate the CMGs
  //Idle the ESCs
  //Center the servos
  //Zero the ESCs
  //Zero the Servos
  //
  runMotor(MOTORZERO);
  MotorRunning = false;
  //Set the Servos to Zero
  setServoPos(0, 0);
  //
  //Create a failsafe log entry
  ReadSensors(0);
  Create_Log_Entry(LogStateChange,0);
  //Serial.println("FailSafe");

}

void failsafe_on_state()
{
  //Audio Chirp (Every thirty seconds)
  bleep(1, 20, 3500, -150, 5, 30000);

  //
  //Slow Pulse the LED!
  setSimpleLED(HIGH,1000);

}







//--------------------------------------------------------------------------------------------------------------------------
//    State Machine Transitions
//--------------------------------------------------------------------------------------------------------------------------
//******************************
//            Setup Transitions
//******************************
//
void setup_state_transitions()
{
  //Create the State Machine Transitions
  //Format:  add_transition State FROM, State TO, Event ID, ON transition?? )
  spacecraft.add_transition(&state_startup, &state_idle, IDLE_EVENT, NULL);
  spacecraft.add_transition(&state_idle, &state_ascend, ASCEND_EVENT, NULL);
  spacecraft.add_transition(&state_ascend, &state_stabilize, STABILIZING_EVENT, NULL);
  spacecraft.add_transition(&state_stabilize, &state_ascend, STABLE_ASCEND_EVENT, NULL);
  spacecraft.add_transition(&state_ascend, &state_slowpan, SLOWPAN_EVENT, NULL);
  spacecraft.add_transition(&state_stabilize, &state_slowpan, STABILIZED_SLOWPAN_EVENT, NULL);
  spacecraft.add_transition(&state_ascend, &state_descend, UNEXPECTED_DECEND_EVENT, NULL);
  spacecraft.add_transition(&state_stabilize, &state_descend, STABILIZED_DECEND_EVENT, NULL);
  spacecraft.add_transition(&state_slowpan, &state_stabilize, SLOWPAN_STABILIZE_EVENT, NULL);
  spacecraft.add_transition(&state_slowpan, &state_descend, SLOWPAN_DECEND_EVENT, NULL);
  spacecraft.add_transition(&state_descend, &state_touchdown, TOUCHDOWN_EVENT, NULL);
  spacecraft.add_transition(NULL, &state_failsafe, FAILSAFE_EVENT, NULL);

}

void setMachineState(int myState, boolean forceState) {

  if (lastStateChgMinute != now.minute() || forceState == true) {
    //Record the current machine state so we can log it later!
    currentState = myState;
    spacecraft.trigger(myState);
    lastStateChgMinute = now.minute();
  }

}








