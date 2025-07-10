void BusScan (void)
{
  int8_t DevicesFound = 0;
  for (byte address = 0x40; address < 0x44; ++address) 
  {
    Wire.beginTransmission(address);
    byte error = Wire.endTransmission();
    if (error == 0) 
    {
      if (SetupScan != 1)
      {
        if (address < 16) 
        {
          Serial.println("M,No I2C Devices Found. Board Error!");
        }
        else if (address == 64)
        {
          Serial.println("M,I2C device found.  Control Board at address      0x40");
        }
        else 
        {
          Serial.print("M,I2C device found.  Expansion Board at address 0x"); Serial.println(address, HEX);
        }
        delay(10);
      }
      else
      {
        delay(10);
      }
      ++DevicesFound;
    } 
  }
  numOutputBoards = DevicesFound;
  if (DevicesFound == 0 && SetupScan != 1) 
  {
    Serial.println("M,No I2C boards found !!!!\n");
    digitalWrite(Ledpin, HIGH);
  } 
}

void WriteToMemory(void)
{
  eeprom.init();
  eeprom.write(0, UseSolenoidPWM);
  delay(5);
  eeprom.write(1, FlipperHoldVal);
  delay(5);
  for (int a = 0; a < 64; a++)
  {
    eeprom.write(a + 2, ReceivedSettings[a+2]);
    delay(5);
    eeprom.write(a + 66, ReceivedSettings[a + 66]);
    delay(5);
  }
  ReadFromMemory();
  digitalWrite(Ledpin, HIGH);
  for (int a = 0; a < 3; a++)
  {
    digitalWrite(Ledpin, !digitalRead(Ledpin));
    delay(150);
  }
  eeprom.close();
}

void FactoryReset(void)
{
  UseSolenoidPWM = 1;
  FlipperHoldVal = 33;
  byte ResetnightMode[64] = { 1, 1, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
  byte ResetoutputTimerVal[64] = { 6, 6, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
            
  eeprom.init();
  eeprom.write(0, UseSolenoidPWM);
  delay(5);
  eeprom.write(1, FlipperHoldVal);
  delay(5);
  for (int a = 0; a < 64; a++)
  {
    eeprom.write(a + 2, ResetnightMode[a]);
    delay(5);
    eeprom.write(a + 66, ResetoutputTimerVal[a]);
    delay(5);
  }
  digitalWrite(Ledpin, HIGH);
  for (int a = 0; a < 3; a++)
  {
    digitalWrite(Ledpin, !digitalRead(Ledpin));
    delay(150);
  }
  ReadFromMemory();
  eeprom.close();
}

void ReadFromMemory(void)
{
  eeprom.init();
  UseSolenoidPWM = eeprom.read(0);
  delay(5);
  FlipperHoldVal = eeprom.read(1);
  delay(5);
  for (int a = 0; a < 64; a++)
  {
    nightModeAssigned[a] = eeprom.read(a + 2);
    delay(5);
    outputTimerVal[a] = eeprom.read(a + 66);
    delay(5);
  }
  digitalWrite(Ledpin, HIGH);
  for (int a = 0; a < 2; a++)
  {
    digitalWrite(Ledpin, !digitalRead(Ledpin));
    delay(100);
  }
  eeprom.close();
}