



void dumpEEprom() {
uint32_t ReadPageStartAddress;
unsigned char rdata[16];



  for (ReadPageStartAddress = 0; ReadPageStartAddress < (MaxStartAddress-16); ReadPageStartAddress+=16) // goes from 0 degrees to the end of the array
  { // increment in steps of ten
    rdata[0] = (char)0;
    Read_i2c_eeprom_page(EEPROM_ADDR, ReadPageStartAddress, rdata, 16);
    Serial.print(ReadPageStartAddress);
    Serial.print(" of ");
    Serial.print(MaxStartAddress);
    Serial.print(":  ");
    Serial.write(rdata,15);
    Serial.println();
  }  

}



void wipeEEprom() {
uint32_t WritePageStartAddress;

  for (WritePageStartAddress = 0; WritePageStartAddress < (MaxStartAddress-16); WritePageStartAddress+=16) // goes from 0 degrees to the end of the array
  {
    str.begin();
    str = "!!!!!!!!!!!!!!!!";     
    Write_i2c_eeprom_page(EEPROM_ADDR, WritePageStartAddress, EEPROMBuffer); // whole page is written at once here
  }

}




void WriteEEpromData() {
unsigned char rdata[16];

  //Verify we can still write to the EEProm without over-writing what we've saved!
  if (CurrentPageStartAddress <= MaxStartAddress ) {
    //Write the entry to the external EEprom
    Write_i2c_eeprom_page(EEPROM_ADDR, CurrentPageStartAddress, EEPROMBuffer); // whole page is written at once here
    //Increment the log record counter!
    CurrentPageStartAddress +=  16;  
    delay(10);  //Write cycle time, REQUIRED!

    //Write the last known page address, so we don't over-write it when we start up next time!
    //EEPROM_writeLong(EEPromPageAddress, CurrentPageStartAddress);
    
    //Read_i2c_eeprom_page(EEPROM_ADDR, (CurrentPageStartAddress - 16), rdata, 16);
    //Serial.write(rdata,15);
    //Serial.println();    
  }
}

void padZero(int valIn) {
   if (valIn < 10 && valIn >= 0) {
     writeZero(1);
   }
   str.print(valIn);
}

void padTwoZeros(int valIn) {
      if (valIn < 100 && valIn > 9) {
        writeZero(1);        
      } else if (valIn < 10 && valIn >= 0) {
        writeZero(2);
      } else if (valIn < 0 && valIn > -10) {
        str.print(" ");        
      }
   str.print(valIn);
}

void writeZero(int addZeros) {
unsigned int numZeros;

  for (numZeros = 0; numZeros < addZeros; numZeros++ )
    {
      str.print("0");
    }
}

void Write_i2c_eeprom_page( int deviceaddress, unsigned int eeaddress, char* data) {
  unsigned char i=0;
  unsigned int address;
  address=eeaddress;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)((address) >> 8)); // MSB
  Wire.write((int)((address) & 0xFF)); // LSB
  do{
    Wire.write((byte) data[i]);i++;
  } while(data[i]);
  Wire.endTransmission();
  delay(10); // data sheet says 5ms for page write
}
//
//
//
//
//
void Read_i2c_eeprom_page(int deviceaddress, unsigned int eeaddress,  
                 unsigned char* data, unsigned int num_chars) 
{
  unsigned char i=0;
  Wire.beginTransmission(deviceaddress);
  Wire.write((int)(eeaddress >> 8));   // MSB
  Wire.write((int)(eeaddress & 0xFF)); // LSB
  Wire.endTransmission();
 
  Wire.requestFrom(deviceaddress,num_chars);
 
  while(Wire.available()) data[i++] = Wire.read();

}







