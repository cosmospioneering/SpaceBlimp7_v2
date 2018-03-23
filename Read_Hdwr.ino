
//--------------------------------------------------------------------------------------------------------------------------
//
//
//   This section contains Data Collection subroutines
//
//
//--------------------------------------------------------------------------------------------------------------------------
// ************************************************
// Read the Appropriate Sensors
// ************************************************
void ReadSensors(byte SensorsToRead) {
  //
  //Read the clock
  //GetRTC();
  now = rtc.now();

  if (SensorsToRead & LogGuidance) {
    //Read the Guidance Sensors
    //Serial.print("mpu\t");
    //Serial.print(lastSenseSecond);
    //Serial.print(" ");
    //Serial.println(now.second());

    if (lastSenseSecond != now.second()) {
      //Serial.print(lastSenseSecond);
      //Serial.print(" ");
      //Serial.println(now.second());
      getMPUData();
      calcActivateStabilizer(true);
    }
    ReadCompass3();
    //Track North via the Advanced LED
    //setTargetHeadingLED(errTargetHeading);
    setTargetHeadingLED();
    CycleLEDs();
  }

  
  if (SensorsToRead & LogPower) {
    //Read the Power Sensors
    readCurrent();
  }

  if (SensorsToRead & LogSensor) {
    //Read the Environment Sensors
    //
    //BUT - only read them when we can read acceptible values!
    if (pauseAltSensorReading==false) { ReadBMP280(); }
    //
    if (lastSenseSecond != now.second()) {
      calcFloatStates();
    }
  }
  
  lastSenseSecond = now.second();
}


















//--------------------------------------------------------------------------------------------------------------------------
//
//
//   This section contains general I2C subroutines
//
//
//--------------------------------------------------------------------------------------------------------------------------
void i2cScan()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning...");

  nDevices = 0;
  for (address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknow error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  //wdt_reset(); // confirm to watchdog timer that all is well
  delay(5000);           // wait 5 seconds for next scan

  //wdt_reset(); // confirm to watchdog timer that all is well
  delay(5000);           // wait 5 seconds for next scan
}















// ************************************************
// Read the Power Sensors
// ************************************************
void readCurrent() {
  float shuntvoltage = 0;

  //Main Board
  shuntvoltage = ina219.getShuntVoltage_mV();
  busvoltage = ina219.getBusVoltage_V();
  current_mA = ina219.getCurrent_mA();

  //PWM Ch0 & Ch1
  shuntvoltage = ina2191.getShuntVoltage_mV();
  busvoltage1 = ina2191.getBusVoltage_V();
  current_mA1 = ina2191.getCurrent_mA();

  //PWM Ch2 & Ch3
  shuntvoltage = ina2192.getShuntVoltage_mV();
  busvoltage2 = ina2192.getBusVoltage_V();
  current_mA2 = ina2192.getCurrent_mA();

}


// ************************************************
// Read the Magnetometer
// ************************************************
void ReadCompass2() {
  //float firstBearing = 0;
  //float lastBearing = 0;
  float tempBearing = 0;
  float minBearing = 0;
  float maxBearing = 0;
  compass_scalled_reading();

  //Serial.print("x = ");
  //Serial.println(compass_x_scalled);
  //Serial.print("y = ");
  //Serial.println(compass_y_scalled);
  //Serial.print("z = ");
  //Serial.println(compass_z_scalled);

  compass_heading();
  //Serial.print ("Heading angle = ");
  //Serial.println (bearing);
  tempBearing = bearing;
  minBearing = bearing;
  maxBearing = bearing;

  int numSamples = 10;
  for (int i = 0; i < numSamples; i++) {
    compass_scalled_reading();
    compass_heading();
    //tempBearing = tempBearing + bearing / numSamples;
    tempBearing = tempBearing + bearing;

    if (bearing > maxBearing)  maxBearing = bearing;
    if (bearing < minBearing)  minBearing = bearing;
    delay(3);
  }
  //bearing = (tempBearing / numSamples);
  tempBearing = (minBearing + maxBearing) / 2;
  //Serial.print ("    Avg Heading angle = ");
  //Serial.println (bearing);
  bearing = tempBearing;
  //Serial.print ("AvgMinMaxBearing = ");
  //Serial.println (tempBearing);

}
//
//
void ReadCompass3() {
  compass_scalled_reading();
  compass_heading();

}



// ************************************************
// Read the Env Sensors
// ************************************************
/*
void ReadBMP180() {
int intAltitude;

  sensors_event_t event;
  bmp.getEvent(&event);

  if (event.pressure) {
    pressure = event.pressure;
    //
    bmp.getTemperature(&temperature);
    //
    float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;
    altitude = bmp.pressureToAltitude(seaLevelPressure, event.pressure);

    //Now let's smooth the readings
    //Convert the altitude to an integer value
    intAltitude = (int)altitude;
    ema = EMA_function(ema_a, intAltitude, ema);
    ema_ema = EMA_function(ema_a, ema, ema_ema);
  }
  else
  {
    //Error!
  }
}
*/



















// ************************************************
// Read the IMU
// ************************************************
void getMPUData(){

  //get the Gyrometer's Yaw axis spin rate
  GyroZ = getMPU_GyZ();
  //Test the reading to see if it is below our threshold (meaning it isn't a crazy value)
  if (abs(GyroZ) < MAX_GYRO_THRESHOLD) {
    //Smooth the reading into our previously smoothed reading
    GyroTurnRate = (GyroTurnRate * .75) + (GyroZ * .25);
  }

  unsigned long nowMillis = millis();
  //get the Accelerometer's Vertical Ascent/Descent rate
  AccelZ = getMPU_AcZ();
  smoothedAccelZ = (smoothedAccelZ * .75) + (AccelZ * .25);
    //We need to multiply the acceleration by the time that has elapsed and the speed of gravity
    //amount of time represented in seconds.milliseconds that has elapsed multiplied by 9.8m/s/s
    //float thisAccelRate = AccelZ * (((nowMillis - lastAscentMillis) / 1000) * 9.8);
    float thisAccelRate = smoothedAccelZ * ((nowMillis - lastAscentMillis) * 0.0098);
    //
    //20m/s/s, just more than twice the speed of gravity!
    //We should never exceed this speed, unless the parachute fails
    if (abs(thisAccelRate) < 20) {
      //Smooth the reading into our previously smoothed reading
      AccelAscentRate = (AccelAscentRate * .75) + (thisAccelRate * .25);
    }
  // save the last time we read the accelerometer
  lastAscentMillis = nowMillis;
}





int16_t  getMPU_GyZ(){
int MPU_addr=0x69;  // I2C address of the MPU-6050
int16_t GyZ;
int16_t rateZ;

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x47);  // starting with register 0x47 (GYRO_ZOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,2,true);  // request a total of 2 registers
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    //gyro_corrected = (float)((gyro_reading/131) - gyro_offset);  // 131 is sensivity of gyro from data sheet
    rateZ = (GyZ / 131) - gyro_offset ;
  return rateZ;

/*
                         raw_value
  output_value = -------------------------     -    offset value
                    sensitivity_setting
*/
}


float  getMPU_AcZ(){
int MPU_addr=0x69;  // I2C address of the MPU-6050
int16_t AcZ;
float rateZ;

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3F);  // starting with register 0x3F (ACCEL_ZOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,2,true);  // request a total of 2 registers
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    rateZ = (AcZ / 16384.0) - accel_offset ;
  return rateZ;

/*
                         raw_value
  output_value = -------------------------     -    offset value
                    sensitivity_setting
*/
}



/*
void getMPU() {
int MPU_addr=0x69;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)     
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
}
*/




void ReadBMP280() {
int intAltitude;
float tmpPressure;
float ema_a = 0.1;

  temperature = bme.readTemperature();
  tmpPressure = bme.readPressure();
  pressure = tmpPressure/100;
  if (pauseAltSensorReading == false) {
    altitude = bme.readAltitude(1013.25);
    //Now let's smooth the readings
    //Convert the altitude to an integer value
    if (decendingFlag == true) {
      altSimulator = altSimulator - 1;
    } else {
      altSimulator = altSimulator + 1;
    }
    intAltitude = (int)altitude + altSimulator;
    ema = EMA_function(ema_a, intAltitude, ema);
    ema_ema = EMA_function(ema_a, ema, ema_ema);
  }
  /*
    Serial.print("BMP280:  ");
    Serial.print(temperature);
    Serial.print("    ");
    Serial.print(pressure);
    Serial.print("    ");
    Serial.print(altitude);
    Serial.print("    ");
    Serial.print(intAltitude);
    Serial.print("    ");
    Serial.print(ema);
    Serial.print("    ");
    Serial.println(ema_ema);
  */
}

int EMA_function(float alpha, int latest, int stored){
  return round(alpha*latest) + round((1-alpha)*stored);
}
