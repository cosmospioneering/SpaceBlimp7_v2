



//--------------------------------------------------------------------------------------------------------------------------
//
//
//   This section contains feedback methods
//
//
//--------------------------------------------------------------------------------------------------------------------------
void bleep(byte numBleeps, byte numTones, word bgnFreq, int stpFreq, int milDelay, long repeatDelay) {

  if (repeatDelay != 0)
  {
    unsigned long nowMillis = millis();

    if (nowMillis - lastSoundMillis < repeatDelay && repeatDelay > 0) {
      //We don't want to make a sound yet, so cancel out the sound!
      numBleeps = 0;
    } else {
      // save the last time you made a sound!
      lastSoundMillis = nowMillis;
    }
  }

  //This function is written so different styles of bleeps can be played, especially via a script, etc...
  for (int j = 0; j < numBleeps; j++)
  {
    for (int i = 0; i < numTones; i++)
    {
      tone(speakerPin, (bgnFreq + (stpFreq * i)));
      delay(milDelay);
    }
  }
  //Where are we sending the tone information?
  noTone(speakerPin);

}

