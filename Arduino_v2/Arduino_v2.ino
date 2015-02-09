//This program powers the brain of our beautiful Natcar.
#include "TimerOne.h"

int linecamValues[128];
int toggle = 0;
byte counter = 0;
int MINEXPOSURE = 300;
int MAXEXPOSURE = 1000;
int exposurecycles = 500 << 1;
int exposuretime = 0;
int SI=0;
int enable = 0;
int cycleflag = 0;
int notKilled = 1;
int lastKillSwitchValue = 1;
long sum = 0;
long avg = 0;

const int pinSI = 10;
const int pinCK = 9;
const int pinAO = 4;
const int pinMotor = 5;
const int pinServo = 3;
const int pinFastStop = 12;
const int pinKillHigh = 4;
const int pinKillSwitch = 2;

/* Control constants */
double Kp = 0.7;
double Kp1 = 0.7;
double Ki = 0.0;
double Ki1 = 0.1;
double Kd = 0.03;
double Kd1 = 0.03;
int killTimeout = 100;
double Scale = 1;
int integrated = 0;
double ServoAmt = 46;
double lastServoOffset = 0;
double MotorSpd = 127;
double lastMotorSpd = 127;
unsigned long lastTime;
int lastLocationDoubled = 0;
int currentLocationDoubled = 0;
int lastDerivative = 0;
int numpossibleLocations = 0;
int possibleLocationsDoubled[65];
int speedMode = 0;



char disp[17]=" .,-:;|=7102348M";


void setup()
{
  //killswitch
  pinMode(pinKillSwitch, INPUT);
  pinMode(pinKillHigh, OUTPUT);
  pinMode(0, OUTPUT);
  digitalWrite(pinKillHigh,HIGH);
  
  pinMode(pinCK, OUTPUT);
  pinMode(SI, OUTPUT);
  Timer1.initialize(50);        // initialize timer1, and set a 1ms second period for a 2 ms period clockd
  Timer1.attachInterrupt(callback);  // attaches callback() as a timer overflow interrupt
  Serial.begin(115200);
  
  //servo and motor
  pinMode(pinServo, OUTPUT);
  pinMode(pinMotor, OUTPUT);
  pinMode(pinFastStop, OUTPUT);
  TCCR2B = TCCR2B & 0b11111000 | 0x06; //register values for frequency 122.0703125 on timer connected pin 3
  analogWrite(pinServo,46); //1.5ms pulse width (actually 93.8/2). 2ms is 125/2 and 1ms is 62.5/2
}

//This function runs every 1ms to gather input from the linecam
void callback()
{
  digitalWrite(pinCK, toggle); //clock output
  exposuretime++;
  if (!toggle)
  {
    if (enable & !cycleflag) //get a analog value from the cam
    {
      linecamValues[counter] = analogRead(4);
      
      sum += linecamValues[counter];
      Serial.print(disp[linecamValues[counter]>>6]);
      //Serial.print(",");
      counter=(counter+1)&127;
    }
    if (SI) //the output to reset the cam for the next frame
    {
      SI = 0;
      digitalWrite(pinSI, 0);
    }
    if (exposuretime >= exposurecycles) //when to end the current exposure and reset the camera
    {
      SI = 1;
      digitalWrite(pinSI, 1);
      enable = 1;
      exposuretime = 0;
      counter = 0;
      sum = 0;
    }
  }
  if (!enable && toggle) //adjust exposure time at the end of a frame capture
  {
    if (sum > 80000 && exposurecycles > MINEXPOSURE)
    {
      exposurecycles-=10;
    }
    if (sum < 52000 && exposurecycles < MAXEXPOSURE)
    {
      exposurecycles+=10;
    }
    
    cycleflag = 1; //allow loop() to use the camera data
    
  }
  toggle = (toggle^1) & enable;
  enable = counter || SI;
}

//Put everything together!  
void loop()
{
  int killSwitchValue = digitalRead(2); //stop the car if the switch is flipped
  if (!killSwitchValue)
    notKilled = 0;
  delay(10);
  //Serial.print(enable);
  //Serial.println(cycleflag);
  if (!enable && cycleflag && notKilled)
  {
    cycleflag = 0;
    
    //compute possible positions of the line
    avg = (sum>>7)+(sum>>8)+(sum>>9)+(sum>>10); //threshold for line detection
    numpossibleLocations = 0;
    if (linecamValues[0] > avg)
    {
      possibleLocationsDoubled[1] = 0;
      numpossibleLocations = 1;
    }
    int i = 1;
    int k = 0;
    while (i<128)
    {
      if (linecamValues[i] > avg)
      {
        if (linecamValues[k] > avg)
        {
          possibleLocationsDoubled[numpossibleLocations]++;
        }
        else
        {
          numpossibleLocations++;
          possibleLocationsDoubled[numpossibleLocations] = i<<1;
        }
      }
      k = i;
      i++;
    }
    if (numpossibleLocations==0 || ((currentLocationDoubled - lastLocationDoubled) > 70 && abs(lastDerivative)/128*14 > 200)) //if the line deviates too much or the line cant be detected
    {
      analogWrite(pinMotor,50);
      if (notKilled < killTimeout-1)
      {
        speedMode = 0;
        if (currentLocationDoubled > 127)
        {
          currentLocationDoubled = 255;
        } else {
          currentLocationDoubled = 0;
        }
      }
      if (notKilled>0)
      {
        notKilled--;
      } else
      {
        notKilled = 0;
      }
    } else {
      notKilled += killTimeout/10;
    }
    int diff = 256;
    int newDiff;
    int possibleLocationDoubled;
    while (numpossibleLocations>0) //if there are multiple lines (bright areas) detected, choose the line closest to the position of the previous line
    {
      possibleLocationDoubled = possibleLocationsDoubled[numpossibleLocations];      
      newDiff = abs(possibleLocationDoubled-lastLocationDoubled);
      if (newDiff<diff)
      {
        diff = newDiff;
        currentLocationDoubled = possibleLocationDoubled;
      }
      numpossibleLocations--;
    }
    
    
    
    //do pid, add servo angle
    int centeredCurrentLocation = currentLocationDoubled - 127;
    int centeredLastLocation = lastLocationDoubled - 127;
    unsigned long currentTime = micros();
    int timeDiff = currentTime - lastTime;
    lastTime = currentTime;
    double derivative = (centeredLastLocation-centeredCurrentLocation)*(1000000.)/timeDiff;
    double derivativeFactor;
    
    integrated = integrated*0.99 + centeredLastLocation*(timeDiff/1000000.);
    if (integrated > 6272)
    {
      integrated = 6272;
    } else if (integrated < -6272)
    {
      integrated = -6272;
    }
    
    double ServoOffset;
    
    switch (speedMode) //switch speeds for straights vs curves
    {
      case 0:
      MotorSpd = 80;
      ServoOffset = Scale*(Kp*centeredCurrentLocation/128*14 + Kd*derivative/128*14 + Ki*integrated/128/99*14);
      break;
      case 1:
      MotorSpd = 140;
      ServoOffset = Scale*(Kp1*centeredCurrentLocation/128*14 + Kd1*derivative/128*14 + Ki1*integrated/128/99*14);
      break;
    }

    ServoAmt = 46+ServoOffset; //46 is the center position
    
    
    if (ServoAmt>60)
    {
      ServoAmt = 60;
    } else if (ServoAmt<32)
    {
      ServoAmt = 32;
    }
    
    if (abs(ServoOffset) < 8 && lastServoOffset < 8 && abs(derivative/128*14)<25)
    {
      speedMode = 1;
    } else {
      speedMode = 0;
    }
    
    if (notKilled < killTimeout)
    {
      MotorSpd = 0;
      ServoAmt = 0;
    }
	
    Serial.println(round(MotorSpd));
    //calc desired speed
    //set servo and motor
    analogWrite(pinServo,round(ServoAmt));
    //digitalWrite(11,HIGH);
    analogWrite(pinMotor,round(MotorSpd));
    if ((MotorSpd - lastMotorSpd)>100 ||
    (MotorSpd > 100 && (lastServoOffset<=11 && abs(ServoOffset)>11 || abs(derivative/128*14) > 100)) ||
    notKilled == killTimeout-1 || notKilled == killTimeout-2)
    {
      digitalWrite(pinFastStop,HIGH);
    } else {
      digitalWrite(pinFastStop,LOW);
    }
    
    lastServoOffset = abs(ServoOffset);
    lastDerivative = derivative;
    lastLocationDoubled = currentLocationDoubled;
  } else if (!notKilled)
  {
    if (killSwitchValue && !lastKillSwitchValue)
    {
      notKilled = killTimeout;
    }
    analogWrite(pinMotor,0);
    analogWrite(pinServo,46);
  }
  lastKillSwitchValue = killSwitchValue;
  lastMotorSpd = MotorSpd;
}
