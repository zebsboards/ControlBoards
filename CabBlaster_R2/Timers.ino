int availableOutputs = 16 + (numOutputBoards * 16);

void CheckOutputTimes (void)
{
  for (int r = 0; r < availableOutputs; r++)
  {
    if (outputTimerVal[r] != 0)
    {
      if ((previousOutputState[r] > 0) && (millis() - outputResetTimer[r] > (outputTimerVal[r] * 60)))
      {
        if (r < 16)
        {
          if (r == fLpos || r == fRpos)
          {

          }
          else if ( r == ShkrPos)
          {
            ledcWrite(ShakerCh, 0);
          }
          else 
          {
            digitalWrite(directOutputs[r], LOW);
          }
        }
        else if (r < 32)
        {
          Chip0.setChannelOff(r - 16); 
        }
        else if (r < 48)
        {
          Chip1.setChannelOff(r - 32);  
        }
        else
        {
          Chip2.setChannelOff(r - 48); 
        }
        if (r != fLpos && r != fRpos){
          previousOutputState[r] = 0;
          outputResetTimer[r] = 0; 
        }
      }
    }
  }
}
