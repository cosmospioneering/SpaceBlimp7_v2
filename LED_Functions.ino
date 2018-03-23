
void CycleLEDs() {

  //capture the current setup setting
  if (StoredSetup != LEDSetup) {
    StoredSetup = LEDSetup;
    //Reset the pattern
    LEDindex = 0;
    LEDState = 0;
  }

  if (StoredSetup & B00000001) {   
    if (LEDOutput == 0) {
      //Initialize the pattern
      LEDOutput = B00000001;
    }
    if (StoredSetup & B01000000) {  
      if (LEDState == 1) {
        //Turn off the LED's
        LEDState = 0;
        //Exit without engaging the pattern logic
        return;
      }
      else
      {
        LEDState = 1;
      }
    }
    else
    {
      LEDState = 1;
    }
    //Cylce the LEDs!
    if (StoredSetup & B00000010) {
      //Run the pattern...  But which one?
      if (StoredSetup & B00000100) {
        //Run the Single Pattern

        //byte LEDSetup = 0;
        //1   = On/Off
        //2   = Pattern/All
        //4   = Single/Alternates
        //8   = PingPong/Forward
        //16  = Mirror/Entire
        //32  = Flip Direction
        //64  = Blink/Normal
        //128 = Invert/Normal
        if (StoredSetup & B00010000) {
          if (StoredSetup & B00001000) {   
            patternCount = 6;
            pingpoingPoint = 3;
          }
          else
          {
            patternCount = 4;
            pingpoingPoint = 16;
          }
        }
        else
        {
          if (StoredSetup & B00001000) {   
            patternCount = 14;
            pingpoingPoint = 7;
          }
          else
          {
            patternCount = 8;
          }
        }


        //Re-initialize the pattern
        LEDOutput = B00000001;
        if (LEDindex > 0) {
          //Do we need to ping-pong these?
          if (StoredSetup & B00001000) {   
            if (LEDindex > pingpoingPoint) {
              //Shift the bit/LED the other direction
              LEDOutput = B10000000;
              LEDOutput = LEDOutput >> (LEDindex - pingpoingPoint);
            }
            else
            {
              //Shift the bit/LED
              LEDOutput = LEDOutput << LEDindex;
            }
          }
          else
          {
            //Shift the bit/LED
            LEDOutput = LEDOutput << LEDindex;
          }
        }

        if (StoredSetup & B00010000) {
          //In order to accomplish the reverse effect,
          //we will load the byte in reverse in the shift-out process
          //Unfortunately, that has no effect on the mirror, so we need to deal with that here
          //Drop the upper four bits
          if (StoredSetup & B00100000) {
            LEDOutput = LEDOutput & B00001111;
            //Shift the lower four bits upstream
            LEDOutput = LEDOutput << 4; 
            if (LEDindex > pingpoingPoint) {
              LEDOutput = B10000000;
              LEDOutput = LEDOutput >> (LEDindex - pingpoingPoint);
            }
            LEDOutHold = LEDOutput;
            mirIndex = 0;
            for (LEDOutTemp = B00001000; LEDOutTemp>0; LEDOutTemp >>= 1) { //iterate through bit mask
              mirIndex +=1;
              //Move the LSB into position and Add it to the output
              LEDOutput = ((LEDOutHold >> (((mirIndex*2))-1)) & LEDOutTemp) | LEDOutput;          
            }
          }
          else
          {
            if (LEDindex > pingpoingPoint) {
              LEDOutput = B00001000;
              LEDOutput = LEDOutput >> (LEDindex - pingpoingPoint);
            }
            LEDOutput = LEDOutput & B00001111;
            mirIndex = 0;
            for (LEDOutTemp = B10000000; LEDOutTemp>8; LEDOutTemp >>= 1) { //iterate through bit mask
              mirIndex +=1;
              //Move the LSB into position and Add it to the output
              LEDOutput = ((LEDOutput << ((8-(mirIndex*2))+1)) & LEDOutTemp) | LEDOutput;          
            }
          }
        }
      }
      else
      {
        //Run the Alternates Pattern
        patternCount = 2;
        if (LEDindex > 0) {
          //XOR the Output so it flips
          LEDOutput = LEDOutput ^ B11111111;
        }
        else
        {
          LEDOutput = B01010101;
        }
      }
    }
    else
    {
      //Turn on all the LEDs!
      //LEDOutput = B11111111;  //All On!      
    }
    //Prepare to write the data!
    if (StoredSetup & B10000000) {
      //XOR and invert the output!
      LEDOutput = LEDOutput ^ B11111111;
    }
    if (StoredSetup & B00100000) {
      setLEDArray(MSBFIRST, LEDOutput);
    }
    else
    {
      setLEDArray(LSBFIRST, LEDOutput);
    }
    LEDindex++;
    if (LEDindex >= patternCount){
      LEDindex = 0;
    }
  }
  else
  {
    //Turn off all the LEDs!
    LEDOutput = B00000000;  //All Off!      
    setLEDArray(LSBFIRST, LEDOutput);
  }
}


void setSimpleLED(int stateLED, long blinkDelay) {
  if (blinkDelay > 0)
  {
    unsigned long nowMillis = millis();

    if (nowMillis - lastSoundMillis >= blinkDelay) {
       // save the last time you blinked the LED
       lastSoundMillis = nowMillis;

       // if the LED is off turn it on and vice-versa:
       if (ledState == LOW) {
         ledState = HIGH;
       } else {
         ledState = LOW;
       }
    } else {
      ledState = stateLED;
    }

    digitalWrite(simpleLEDPin, ledState);

  } else { digitalWrite(simpleLEDPin, ledState); }
}


void setLEDArray(byte bitOrder, byte LEDArray) {
  //LEDs start @ channel 4 and end at channel 11
  uint16_t i = 12;

  bitOrder = MSBFIRST;
  
  if (bitOrder == MSBFIRST) {
    for (LEDOutTemp = B10000000; i > 4; LEDOutTemp >>= 1) { //iterate through bit mask
      i += -1;
      setLEDstate(i, LEDArray, LEDOutTemp);
    }
  }
  else
  {
    for (LEDOutTemp = B00000001; i > 4; LEDOutTemp <<= 1) { //iterate through bit mask
      i += -1;
      setLEDstate(i, LEDArray, LEDOutTemp);
    }
  }
}


void setLEDstate(uint16_t LEDChannel, byte LEDOut, byte LEDTestMask) {
    if (LEDOut & LEDTestMask) {
      pwm.setPWM(LEDChannel, 0, 1024);
    }
    else
    {
      pwm.setPWM(LEDChannel, 0, 0);
    }
}





void setTargetHeadingLED() {

  double errHeading = abs(targHeading - bearing);
  
  //Turn on the LEDs
  LEDSetup = B00000001;
  //LEDSetup = B00000000;

  if (targHeading < bearing) {
    valDirection = -1;
  }
  else
  {
    valDirection = 1;
  }

  if (errHeading > 180) {
    //Motor must be spinning CCW
    //This result means the target position is closer if we spin in the opposite direction
    errHeading = (360 - errHeading);
    valDirection = valDirection * -1;
    //Serial.print("alt");
  }
  errHeading = errHeading * valDirection;
  //Serial.print("(");
  //Serial.print(targHeading);
  //Serial.print(" - ");
  //Serial.print(bearing);
  //Serial.print(") * ");
  //Serial.print(valDirection);
  //Serial.print(" * -1 = ");
  //Serial.println(errHeading);


  //Pointing right at the target
  if (errHeading > -5 && errHeading < 5) {
    LEDOutput = B00011000;
  }

  //First quadrants left and right of the front center
  if (errHeading >= 5 && errHeading < 30) {
    LEDOutput = B00010000;
  }
  if (errHeading > -30 && errHeading <= -5) {
    LEDOutput = B00001000;
  }

  //Second quadrants left and right of the front center
  if (errHeading >= 30 && errHeading < 60) {
    LEDOutput = B00100000;
  }
  if (errHeading > -60 && errHeading <= -30) {
    LEDOutput = B00000100;
  }


  //Third quadrants left and right of the front center
  if (errHeading >= 60 && errHeading < 90) {
    LEDOutput = B01000000;
  }
  if (errHeading > -90 && errHeading <= -60) {
    LEDOutput = B00000010;
  }


  //Fourth quadrants left and right of the front center
  if (errHeading >= 90 && errHeading < 135) {
    LEDOutput = B10000000;
  }
  if (errHeading > -135 && errHeading <= -90) {
    LEDOutput = B00000001;
  }
  if (errHeading <= -135 || errHeading >= 135) {
    LEDOutput = B10000001;
  }

}



