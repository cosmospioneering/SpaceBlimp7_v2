
// ************************************************
// Create a Log Entry
// ************************************************
void Create_Log_Entry(byte entriesToCreate, int interval)
{
boolean writeLog = true;
boolean calcLog = false;
int remainder = now.minute() % 10;
uint8_t wholeVoltTemp,fracVoltTemp;


  if (interval == 1 && lastMinute != now.minute()) {
    writeLog = true;
    writeLog = true;
  } else if (interval == 0) {
    writeLog = true;
  } else {
    return;
  }



  
  //Verify we still have room to write log entries!
  //



  //Verify we are on mission day!
  //
  //if (now.month() == 3 || now.day() != 24) { writeLog = false; }
  


  //Build & store the log entries!
  if (entriesToCreate & LogStateChange || lastHour != now.hour()) {
    //This is a state change log entry
    str.begin();
    //0 bytes
    str = "M";  //"M" signifies machine state change     
    //1 byte
    padZero(currentState);
    //3 bytes
    //Now capture the full date/time stamp
    padZero(now.year());
    //7 bytes
    padZero(now.month());
    //9 bytes
    padZero(now.day());
    //11 bytes
    padZero(now.hour());
    //13 bytes
    padZero(now.minute());
    //15 bytes
    //str.print("!");
    //16 bytes

    lastHour = now.hour();

    if (writeLog == true) {
      //
      //Write the entry to the external EEprom
      WriteEEpromData();
    }
    else {
      Serial.println(str);
    }
  }
  
  if (entriesToCreate & LogPower) {
    //This is an "C" Power record
    //
    //Construct second char string of 28 bytes to complete the record
    /*
            C Record
      Date is known, so only time-minutes is needed:
    */
    str.begin();
    //0 bytes
    str = "B";  //"B" signifies critical power & sensor data record
    //1 byte
    padZero(now.minute());
    //3 bytes
    wholeVoltTemp = (int)busvoltage; fracVoltTemp= (busvoltage - wholeVoltTemp) * 10;
    if (wholeVoltTemp > 0 && wholeVoltTemp < 5) {
      str.print(wholeVoltTemp);
      //if (fracVoltTemp < 10) { str.print("0"); }
      str.print(fracVoltTemp);
      if (current_mA > 0) {
        padTwoZeros((int)current_mA);
      } else {
        writeZero(3);
      }
    } else {
        writeZero(5);
    }
    //8 bytes

    padTwoZeros((int)temperature);
    //11 bytes

    if (pressure < 1000) { writeZero(1); }
    padTwoZeros((int)pressure);
    //15 bytes

    //str.print("!");
    //16 bytes

      
    if (writeLog == true) {
      //
      //Write the entry to the external EEprom
      WriteEEpromData();
    }
    else {
      Serial.println(str);
    }
     

      
    str.begin();
    //0 bytes
    str = "S";  //"S" signifies secondary power data record
    //1 byte
    padZero(now.minute());
    //3 bytes
    wholeVoltTemp = (int)busvoltage1; fracVoltTemp= (busvoltage1 - wholeVoltTemp) * 10;
    if (wholeVoltTemp > 0 && wholeVoltTemp < 5) {
      str.print(wholeVoltTemp);
      //if (fracVoltTemp < 10) { str.print("0"); }
      str.print(fracVoltTemp);
      if (current_mA1 > 0) {
        padTwoZeros((int)current_mA1);
      } else {
        writeZero(3);
      }
    } else {
        writeZero(5);
    }
    //8 bytes

    wholeVoltTemp = (int)busvoltage2; fracVoltTemp = (busvoltage2 - wholeVoltTemp) * 10;
    if (wholeVoltTemp > 0 && wholeVoltTemp < 5) {
      str.print(wholeVoltTemp);
      //if (fracVoltTemp < 10) { str.print("0"); }
      str.print(fracVoltTemp);
      if (current_mA2 > 0) {
        padTwoZeros((int)current_mA2);
      } else {
        //str.print("000");
        writeZero(3);
      }
    } else {
      //str.print("00000");
      writeZero(5);
    }
    //13 bytes
    str.print("!!!");
    //16 bytes
             
    if (writeLog == true) {
      //
      //Write the entry to the external EEprom
      WriteEEpromData();
    }
    else {
      Serial.println(str);
    }
  }



  
  if (entriesToCreate & LogSensor) {
    
    //Finish calculating the altitude trends
    //calcFloatStates();
    
    /*
            G Record
      Date is known, so only time is needed:
    */
    //This is a "G" Guidance record
    //
    str.begin();
    //0 bytes
    str = "E";  //"E" signifies environment record
    //1 byte1
    padZero(now.minute());
    //3 bytes
    
    padTwoZeros((int)bearing);
    //6 bytes


    padTwoZeros(GyroTurnRate);
    //9 bytes

    if (altDEMA < 10000 && altDEMA > 0) { writeZero(1); }
    if (altDEMA < 1000 && altDEMA > 0) { writeZero(1); }
    padTwoZeros(altDEMA);
    //15 bytes
    str.print("!");
    //16 bytes

    if (writeLog == true) {
      //
      //Write the entry to the external EEprom
      WriteEEpromData();
    }
    else {
      Serial.println(str);
    }

      
      
    /*
            C Record
      Date is known, so only time is needed:
    */
    //This is a "C" Control record
    //
    str.begin();
    //0 bytes
    str = "C";  //"C" signifies Control systems record
    //1 byte
    padZero(now.minute());
    //3 bytes

    padTwoZeros((int)targHeading);
    //6 bytes

    //Servo position
    padTwoZeros(posLast);
    //9 bytes

    //Motor PWM setting
    padTwoZeros(LastMOTOR);
    //12 bytes

    padZero(resetServoCount);
    //14 bytes
    str.print("!!");
    //16 bytes
    
    if (writeLog == true) {
      //
      //Write the entry to the external EEprom
      WriteEEpromData();
    }
    else {
      Serial.println(str);
    }
  }



  
  if (entriesToCreate & LogCMGPosCal) {
    str.begin();
    //0 bytes
    str = "P";
    //1 byte
    str.print(",");
    //Servo position
    padZero((int)idxGimbalAngle);
    //4 bytes
    str.print(",");
    padZero((int)posDelay);
    //7 bytes
    str.print(",");
    padTwoZeros(storedSpinRate);
    //11 bytes
    str.print(",");
    padTwoZeros(GyroTurnRate);
    //15 bytes
    
    if (writeLog == true) {
      //
      //Write the entry to the external EEprom
      WriteEEpromData();
    }
    else {
      Serial.println(str);
    }    
  }

  if (entriesToCreate & LogCMGNegCal) {
    str.begin();
    //0 bytes
    str = "N";
    //1 byte
    str.print(",");
    //Servo position
    padZero((int)idxGimbalAngle);
    //4 bytes
    str.print(",");
    padZero((int)posDelay);
    //7 bytes
    str.print(",");
    padTwoZeros(storedSpinRate);
    //11 bytes
    str.print(",");
    padTwoZeros(GyroTurnRate);
    //15 bytes

    if (writeLog == true) {
      //
      //Write the entry to the external EEprom
      WriteEEpromData();
    }
    else {
      Serial.println(str);
    }    
  }
    


  lastMinute = now.minute();
}
































