   #include<etkinBotPid.h>

/******************************************************************************
gerekli degiskenler!!!!!!!!!!!!!

*******************************************************************************/

EtkinPidClass::EtkinPidClass()
{
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  attachInterrupt(1, interrupt1, FALLING);
  attachInterrupt(2, interrupt2, FALLING);

  myPID1.SetMode(AUTOMATIC);
  myPID2.SetMode(AUTOMATIC);
  myPID1.SetOutputLimits(0, 255);
  myPID2.SetOutputLimits(0, 255);

}
