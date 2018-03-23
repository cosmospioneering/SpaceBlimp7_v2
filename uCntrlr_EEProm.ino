

//--------------------------------------------------------------------------------------------------------------------------
//
//
//   This section contains general EEProm subroutines
//
//
//--------------------------------------------------------------------------------------------------------------------------
// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
  int sizeEEPROM;
  bleep(1, 10, 3500, -300, 3, false);

  if (compass_x_offset != EEPROM_readFloat(CompassXOffsetAddress))
  {
    sizeEEPROM = EEPROM_writeFloat(CompassXOffsetAddress, compass_x_offset);
  }
  if (compass_y_offset != EEPROM_readFloat(CompassYOffsetAddress))
  {
    sizeEEPROM = EEPROM_writeFloat(CompassYOffsetAddress, compass_y_offset);
  }
  if (compass_z_offset != EEPROM_readFloat(CompassZOffsetAddress))
  {
    sizeEEPROM = EEPROM_writeFloat(CompassZOffsetAddress, compass_z_offset);
  }
}




// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{

  //Serial.println("Loading Compass Offsets");

  //Setup the Compass Variables
  compass_x_gainError = 1.12;
  compass_y_gainError = 1.13;
  compass_z_gainError = 1.03;

  // Load from EEPROM
  compass_x_offset = EEPROM_readFloat(CompassXOffsetAddress);
  compass_y_offset = EEPROM_readFloat(CompassYOffsetAddress);
  compass_z_offset = EEPROM_readFloat(CompassZOffsetAddress);

  //CurrentPageStartAddress = EEPROM_readlong(EEPromPageAddress);


  // Use defaults if EEPROM values are invalid
  if (isnan(compass_x_offset))
  {
    compass_x_offset = -413.98;
  }
  if (isnan(compass_y_offset))
  {
    compass_y_offset = 549.80;
  }
  if (isnan(compass_z_offset))
  {
    compass_z_offset = 83.99;
  }
}



// ************************************************
// Write floating point values to EEPROM
// ************************************************
int EEPROM_writeFloat(int address, float value)
{
int i;

  byte* p = (byte*)(void*)&value;
  for (i = 0; i < sizeof(value); i++)
  {
    EEPROM.write(address++, *p++);
  }
  delay(5000);
  return i;
}// ************************************************
// Read values from Arduino's EEPROM
// ************************************************
float EEPROM_readFloat(int address)
{
  float value = 0.0;
  byte* p = (byte*)(void*)&value;
  for (int i = 0; i < sizeof(value); i++)
  {
    *p++ = EEPROM.read(address++);
  }
  return value;
}

/*
unsigned int EEPROM_readInt(int address)
 {
   byte lowByte = EEPROM.read(address);
   byte highByte = EEPROM.read(address + 1);
  
   return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
 }
void EEPROM_writeInt(int address, int p_value)
     {
     byte lowByte = ((p_value >> 0) & 0xFF);
     byte highByte = ((p_value >> 8) & 0xFF);

     EEPROM.write(address, lowByte);
     EEPROM.write(address + 1, highByte);
     }
*/

long EEPROM_readlong(int address)
{
      //Read the 4 bytes from the eeprom memory.
      long four = EEPROM.read(address);
      long three = EEPROM.read(address + 1);
      long two = EEPROM.read(address + 2);
      long one = EEPROM.read(address + 3);

      //Return the recomposed long by using bitshift.
      return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}
void EEPROM_writeLong(int address, long value)
{
      //Decomposition from a long to 4 bytes by using bitshift.
      //One = Most significant -> Four = Least significant byte
      byte four = (value & 0xFF);
      byte three = ((value >> 8) & 0xFF);
      byte two = ((value >> 16) & 0xFF);
      byte one = ((value >> 24) & 0xFF);

      //Write the 4 bytes into the eeprom memory.
      EEPROM.write(address, four);
      EEPROM.write(address + 1, three);
      EEPROM.write(address + 2, two);
      EEPROM.write(address + 3, one);
}



