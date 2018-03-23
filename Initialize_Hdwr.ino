




//--------------------------------------------------------------------------------------------------------------------------
//
//
//   This section contains Hardware initialization subroutines
//
//
//--------------------------------------------------------------------------------------------------------------------------
//
// ************************************************
// Initialize the Power Sensors
// ************************************************
void initMeter() {
  //Serial.println("Init INA219");
  ina219.begin();
  ina2191.begin();
  ina2192.begin();
  //ina219.ina219SetCalibration_32V_1A();
  //ina219.ina219SetCalibration_16V_400mA();
  //ina2191.ina219SetCalibration_16V_400mA();
  //ina2192.ina219SetCalibration_16V_400mA();
}


// ************************************************
// Initialize the Real Time Clock
// ************************************************
void initRTC() {

  rtc.begin();
  
  //if (! rtc.begin()) {
    //Serial.println("No RTC");
    //while (1);
  //}

  //if (rtc.lostPower()) {
    //Serial.println("RTC power fail");
    // following line sets the RTC to the date & time this sketch was compiled
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  //}
  //rtc.adjust(DateTime(2018, 3, 21, 21, 58, 0));
}




// ************************************************
// Initialize the Env Sensor
// ************************************************
void initBMP() {
  
  //bmp.begin();
 bme.begin(0x76);

}



// ************************************************
// Initialize the PWM Controller & ESCs
// ************************************************
void initPWM() {
  pwm.reset();
  pwm.begin();
  pwm.setPWMFreq(50);  // 60 is the maximum PWM frequency for Servos, suggest using 50 instead

  //byte LEDSetup = 0;
  //1   = On/Off
  //2   = Pattern/All
  //4   = Single/Alternates
  //8   = PingPong/Forward
  //16  = Mirror/Entire
  //32  = Flip Direction
  //64  = Blink/Normal
  //128 = Invert/Normal
  LEDSetup = B00000001;
  LEDOutput = B00000000;  //All Off!
  //setLEDArray(LSBFIRST, LEDOutput);
  LEDSetup = B00000001;
}
//



// ************************************************
// Initialize the Compass
// ************************************************
void initCompass() {
  //Serial.println("Init Compass");
  compass_init(2);

  //timerDelay(1000);
  //
  //CompassCalibrationCheck();
  //
}





// ************************************************
// Initialize the MPU6050
// ************************************************
void intMPU(){
int MPU_addr=0x69;  // I2C address of the MPU-6050

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);



/*
  Angular Velocity Limit     |   Sensitivity
  --------------------------------------------
  250º/s      (0)            |    131
  500º/s      (1)            |    65.5 
  1000º/s     (2)            |    32.8 
  2000º/s     (3)            |    16.4
*/
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
  Wire.endTransmission(true);




/*
  Acceleration Limit     |   Sensitivity
  ----------------------------------------
  2g          (0)        |    16,384
  4g          (1)        |    8,192  
  8g          (2)        |    4,096 
  16g         (3)        |    2,048 
*/
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x1C); //Accessing the register 1C - Accelerometer Configuration
  Wire.write(0x00000000); //Setting the accelerometer to full scale +/- 2g
  Wire.endTransmission(true);




/*
  Interrupt Pin Configuration
  ----------------------------------------  
*/
  //Wire.beginTransmission(MPU_addr);
  //Wire.write(0x37); //Accessing the register 1D - Interrupt Enable Configuration
  //Wire.write(0x02); // Set interrupt pin active high, push-pull
  //Wire.endTransmission(true);


/*
  Interrupt Enable Configuration
  ----------------------------------------  
*/
  //Wire.beginTransmission(MPU_addr);
  //Wire.write(0x38); //Accessing the register 1D - Interrupt Enable Configuration
  //Wire.write(0x40); //Setting the Interrupt Enable Config for Motion threshold
  //Wire.endTransmission(true);



/*
  Freefall threshold of |0mg|
*/ 
  //Wire.beginTransmission(MPU_addr);
  //Wire.write(0x1D); //Accessing the register 1D - Freefall Threshold Configuration
  //Wire.write(0x00000000); //Setting the freefall threshold to  +/- 0mg
  //Wire.endTransmission(true);

/*
  Freefall duration limit of |0ms|
*/
  //Wire.beginTransmission(MPU_addr);
  //Wire.write(0x1E); //Accessing the register 1E - Freefall Duration Configuration
  //Wire.write(0x00000000); //Setting the freefall duration to  0s
  //Wire.endTransmission(true);

  // Set up the interrupt pin, its set as active high, push-pull
  //pinMode(intPin, INPUT);
  //digitalWrite(intPin, LOW);



}

