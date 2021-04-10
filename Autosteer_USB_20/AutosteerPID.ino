void calcSteeringPID(void) 
 {  
  //Proportional only
  pValue = steerSettings.Kp * steerAngleError;
  pwmDrive = (int)pValue;
  
  errorAbs = abs(steerAngleError);
  int newMax = 0; 
   
  if (errorAbs < LOW_HIGH_DEGREES)
  {
    newMax = (errorAbs * highLowPerDeg) + steerSettings.lowPWM;
  }
  else newMax = steerSettings.highPWM;
    
  //add min throttle factor so no delay from motor resistance.
  if (pwmDrive < 0 ) pwmDrive -= steerSettings.minPWM;
  else if (pwmDrive > 0 ) pwmDrive += steerSettings.minPWM;
  
  //Serial.print(newMax); //The actual steering angle in degrees
  //Serial.print(",");

  //limit the pwm drive
  if (pwmDrive > newMax) pwmDrive = newMax;
  if (pwmDrive < -newMax) pwmDrive = -newMax;

  if (steerConfig.MotorDriveDirection) pwmDrive *= -1;

  if (steerConfig.isDanfoss)
  {
    // Danfoss: PWM 25% On = Left Position max  (below Valve=Center)
    // Danfoss: PWM 50% On = Center Position
    // Danfoss: PWM 75% On = Right Position max (above Valve=Center)
    pwmDrive = (constrain(pwmDrive, -250, 250));

    // Calculations below make sure pwmDrive values are between 65 and 190
    // This means they are always positive, so in motorDrive, no need to check for
    // steerConfig.isDanfoss anymore
    pwmDrive = pwmDrive >> 2; // Devide by 4
    pwmDrive += 128;          // add Center Pos.
  }
 }

//#########################################################################################

void motorDrive(void) 
  {
    // Write to CANBUS for PVEA-CI
    if (steerConfig.CytronDriver)
    { 
      CAN_message_t msg;
      msg.flags.extended = 1;
      msg.id = 0x0CFE3022;
      for ( uint8_t i = 0; i < 8; i++ ) msg.buf[i] = 0;
      msg.buf[1]=0xFF;
      // Cytron MD30C Driver Dir + PWM Signal
      if (pwmDrive >= 0)
      {
        msg.buf[2]=0xC1;  //set the correct direction
      }
      else   
      {
        msg.buf[2]=0xC2;
        pwmDrive = -pwmDrive;
      }
  
      //write out the 0 to 255 value
      msg.buf[0]=(uint8_t)pwmDrive;
      Can0.write(msg);
      pwmDisplay = pwmDrive;
    }
    else
    {
      // IBT 2 Driver Dir1 connected to BOTH enables
      // PWM Left + PWM Right Signal     
    
      if (pwmDrive > 0)
      {
        analogWrite(PWM2_RPWM, 0);//Turn off before other one on
        analogWrite(PWM1_LPWM, pwmDrive);
      }      
      else
      {
        pwmDrive = -1 * pwmDrive;  
        analogWrite(PWM1_LPWM, 0);//Turn off before other one on
        analogWrite(PWM2_RPWM, pwmDrive);
      }
      
      pwmDisplay = pwmDrive;
    }  
  }
