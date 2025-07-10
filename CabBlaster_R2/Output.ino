byte incomingData[10];
int BankOffset;
int LedToLight;

void ReceiveOutputData(void) 
{
  int dataLocation = 0;
  int bytePosition;
  int counter = 0;

  for (int i = 0; i < 11; i++) 
  {
    incomingData[dataLocation] = Serial.read();
    switch (incomingData[0]) 
    {
      case 'B':                                                 // Number of Boards Found Requested
        Serial.println(numOutputBoards);
        dataLocation = 0;
        i = 10;
      break;
      case 'F':                                                 // Factory Settings Reset
        FactoryReset();
        Serial.println("F");
        dataLocation = 0;
        i = 10;
      break;
      case 'L':                                                 // OnBoard LED Control Test
        for (int a = 0; a < 4; a++)
        {
          digitalWrite(Ledpin, !digitalRead(Ledpin));
          delay(250);
        }
        dataLocation = 0;
        i = 10;
      break;
      case 'O':                                                 // DOF Controlled Data Incoming
        if (dataLocation == 1)  // data sent is always [0](already entered into array)[200 + bank offset][output value 1,value 2,value 3.....value 7]
        {                       // check to make sure we are reading what's expected for the first 2 bytes of data. If not, reset and start the buffer over again
          if ((incomingData[0] = 'O') && (incomingData[1] < 200) || (incomingData[1] > 207)) 
          {
            dataLocation = 1;  // incorrect data - hold until buffer cleared
          } 
          else 
          {
            dataLocation++;  // advance to next byte in array
          }
        } 
        else 
        {
          dataLocation++;  // advance to next byte in array
        }
        if (dataLocation == 10) 
        { 
          SendOutputData();
        }
      break;
      case 'Q':                                                 // Send Settings Data
        Serial.print("P,"); Serial.print(UseSolenoidPWM);
        Serial.println("");
        delay(100);
        while(Serial.read() != 'A'){}
        Serial.print("H,"); Serial.print(FlipperHoldVal);
        Serial.println("");
        delay(100);
        while(Serial.read() != 'A'){}
        Serial.print("N,");
        for (int a = 0; a < 64; a++)
        {
          Serial.print(nightModeAssigned[a]);
          Serial.print(",");
          delay(1);
        }
        Serial.println("");
        delay(100);
        while(Serial.read() != 'A'){}
        Serial.print("T,");
        for (int a = 0; a < 64; a++)
        {
          Serial.print(outputTimerVal[a]);
          Serial.print(",");
          delay(1);
        }
        Serial.println("");
        delay(100);
        for (int a = 0; a < 3; a++)
        {
          digitalWrite(Ledpin, LOW);
          delay(250);
          digitalWrite(Ledpin, HIGH);
          delay(250);
        }
        dataLocation = 0;
        i = 10;
      break;
      case 'R':                                                 // Receive Settings Data
        delay(200);
        while (Serial.available())
        {
          ReceivedSettings[counter] = Serial.read();
          counter++;
        }
        UseSolenoidPWM = ReceivedSettings[0];
        FlipperHoldVal = ReceivedSettings[1];
        for (int a = 2; a < 67; a++)
        {
          nightModeAssigned[a] = ReceivedSettings[a];
        }
        for (int a = 67; a < 131; a++)
        {
          outputTimerVal[a] = ReceivedSettings[a];
        }
        Serial.println("A");
        WriteToMemory();
        dataLocation = 0;
        i = 10;
      break;
      case 'S':                                                 // Scan the i2c bus and Report back
        BusScan();
        dataLocation = 0;
        i = 10;
      break;
      case 'T':                                                 // Test Output Data Incoming
        Serial.println("M,Flashing Outputs Sequentially ......");
        for (int a = 0; a < (numOutputBoards + 1) * 16; a++) 
        {
          if (a == fLpos || a == fRpos || a == ShkrPos)
          {
            ledcWrite(a, 255);
            delay(200);
            ledcWrite(a, 0);
            delay(200);
          }
          else
          {
            if ( a < 16)
            {
              digitalWrite(directOutputs[a], HIGH);
              delay(200);
              digitalWrite(directOutputs[a], LOW);
              delay(200);
            }
            else if (a < 32)
            {
              Chip0.setChannelOn(a - 16);
              delay(200);
              Chip0.setChannelOff(a - 16);
              delay(200);
            }
            else if (a < 48 && numOutputBoards > 1)
            {
              Chip1.setChannelOn(a - 32);
              delay(200);
              Chip1.setChannelOff(a - 32);
              delay(200);
            }
            else if (a < 64 && numOutputBoards > 2)
            {
              Chip2.setChannelOn(a - 48);
              delay(200);
              Chip2.setChannelOff(a - 48);
              delay(200);
            }
          }
        }
        Serial.println("M,Testing Complete");
        i = 10;
        dataLocation = 0;
      break;
      case 'U':                                                 // Controller Verification Requested
        Serial.println("41C5");  // DOF Requests Firmware Version
        dataLocation = 0;
        i = 10;
      break;
      case 'X':                                                 // Report Variable States (Received Settings, Loaded in Memory, Stored in EEPROM)
        Serial.println("M, ");
        Serial.println("M,Recent Values Sent From Config Utility");
        Serial.println("M, ");
        Serial.print("M,Flipper PWM Enabled  ");Serial.println(ReceivedSettings[0]);
        Serial.print("M,Flipper PWM Value  "); Serial.println(ReceivedSettings[1]);
        Serial.print("M,NightMode    ");
        for (int a = 0; a < 63; a++)
        {
          Serial.print(ReceivedSettings[a+2]);
          delay(3);
        }
        Serial.println(ReceivedSettings[63+2]);
        Serial.print("M,Output Timer ");
        for (int a = 0; a < 63; a++)
        {
          Serial.print(ReceivedSettings[a+66]);
          delay(3);
        }
        Serial.println(ReceivedSettings[63+66]);
        Serial.println("M, ");
        Serial.println("M,Loaded In Controller Memory"); 
        Serial.println("M, ");   
        Serial.print("M,Flipper PWM Enabled  ");Serial.println(UseSolenoidPWM);
        Serial.print("M,Flipper PWM Value  "); Serial.println(FlipperHoldVal);  
        Serial.print("M,NightMode    ");  
        for (int a = 0; a < 63; a++)
        {
          Serial.print(nightModeAssigned[a]);
        }
        Serial.println(nightModeAssigned[63]);
        Serial.print("M,Output Timer ");
        for (int a = 0; a < 63; a++)
        {
          Serial.print(outputTimerVal[a]);
          delay(3);
        }
        Serial.println(outputTimerVal[63]);
        Serial.println("M, ");
        Serial.println("M,Stored in EEPROM");
        eeprom.init();
        Serial.println("M, ");
        Serial.print("M,Flipper PWM Enabled  ");Serial.println(eeprom.read(0));
        Serial.print("M,Flipper PWM Value  "); Serial.println(eeprom.read(1)); 
        Serial.print("M,NightMode    ");  
        int advance;
        for (byte a = 2; a < (numOutputBoards  * 16) + 16 + 1; a++)
        {
          Serial.print(eeprom.read(a));
          advance = a;
          delay(3);
        }
        Serial.println(eeprom.read(advance+1));
        Serial.print("M,Output Timer ");
        for (byte a = 66; a < (66 + (numOutputBoards  * 16) + 15); a++)
        {
          Serial.print(eeprom.read(a));
          advance = a;
          delay(3);
        }
        Serial.println(eeprom.read(advance+1));
        eeprom.close();
        dataLocation = 0;
        i = 10;
      break;
    }
  }
}

void SendOutputData(void) {
  
  BankOffset = (incomingData[1] - 200) * 8;  // create bank offset for outputs (0,8,16,24,32 .....56)
  for (int i = 2; i < 10; i++) {
    LedToLight = (i - 2) + BankOffset;  // output location in 64 byte array ((i-2) + 0, (i - 2) + 8, (i-2) + 16 ..... (i-2) + 56 ) = output number in array`
    if (nightModeIn != HIGH && nightModeAssigned[LedToLight] != 0) 
    {
       incomingData[i] = 0; 
    } 
    if (incomingData[i] != previousOutputState[LedToLight]) 
    {
      if (LedToLight < 16) 
      {
        if (incomingData[i] < 128 && LedToLight != ShkrPos) 
        {
          if (LedToLight == fLpos) 
          {
            ledcWrite(FlipperLeftCh, FlipperOffVal);
            fLheld = 0;
          } 
          else if (LedToLight == fRpos) 
          {
            ledcWrite(FlipperRightCh, FlipperOffVal);
            fRheld = 0;
          } 
          else 
          {
            digitalWrite(directOutputs[LedToLight], LOW);
          }
        } 
        else if (incomingData[i] > 127 && LedToLight != ShkrPos) 
        {
          if (LedToLight == fLpos) 
          {
            ledcWrite(FlipperLeftCh, FlipperFireVal);
          } 
          else if (LedToLight == fRpos) 
          {
            ledcWrite(FlipperRightCh, FlipperFireVal);
          } 
          else 
          {
            digitalWrite(directOutputs[LedToLight], HIGH);
          }
        } 
        else if (LedToLight == ShkrPos) 
        {
          ledcWrite(ShakerCh, incomingData[i]);
        } 
      }  
      else if (LedToLight < 32) 
      {
        switch (incomingData[i]) 
        {
          case 0:
            Chip0.setChannelOff(LedToLight - 16);
            break;
          case 255:
            Chip0.setChannelOn(LedToLight - 16);
            break;
          default:
            Chip0.setChannelPWM(LedToLight - 16, incomingData[i] * 16);
            break;
        }
      }
      else if (LedToLight < 48) 
      {
        switch (incomingData[i]) 
        {
          case 0:
            Chip1.setChannelOff(LedToLight - 32);
            break;
          case 255:
            Chip1.setChannelOn(LedToLight - 32);
            break;
          default:
            Chip1.setChannelPWM(LedToLight - 32, incomingData[i] * 16);
            break;
        }
      }
      else if (LedToLight < 64) 
      {
        switch (incomingData[i]) 
        {
          case 0:
            Chip2.setChannelOff(LedToLight - 48);
            break;
          case 255:
            Chip2.setChannelOn(LedToLight - 48);
            break;
          default:
            Chip2.setChannelPWM(LedToLight - 48, incomingData[i] * 16);
            break;
        }
      }
      previousOutputState[LedToLight] = incomingData[i];
      outputResetTimer[LedToLight] = millis();
    }
  }
  
}

void ResetAllOutputs(void) {
  for (int r = 0; r < 16; r++) {
    if (r == fLpos || r == fRpos || r == ShkrPos) {
      ledcWrite(FlipperLeftCh, FlipperOffVal);
      ledcWrite(FlipperRightCh, FlipperOffVal);
      ledcWrite(ShakerCh, FlipperOffVal);
    } else {
      digitalWrite(directOutputs[r], LOW);
    }
    previousOutputState[r] = 0;
    outputResetTimer[r] = 0;
    switch (numOutputBoards) {
      case 1:
        Chip0.setChannelOff(r);
        previousOutputState[r + 16] = 0;
        outputResetTimer[r + 16] = 0;
        break;
      case 2:
        Chip0.setChannelOff(r);
        previousOutputState[r + 16] = 0;
        outputResetTimer[r + 16] = 0;
        Chip1.setChannelOff(r);
        previousOutputState[r + 32] = 0;
        outputResetTimer[r + 32] = 0;
        break;
      case 3:
        Chip0.setChannelOff(r);
        previousOutputState[r + 16] = 0;
        outputResetTimer[r + 16] = 0;
        Chip1.setChannelOff(r);
        previousOutputState[r + 32] = 0;
        outputResetTimer[r + 32] = 0;
        Chip2.setChannelOff(r);
        previousOutputState[r + 48] = 0;
        outputResetTimer[r + 48] = 0;
        break;
      default:
        break;
    }
  }
}
