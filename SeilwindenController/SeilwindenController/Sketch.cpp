/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>
#include "Servo.h"
/*End of auto generated code by Atmel studio */



/*
Sender:    Maverick MTX-242
Empfänger: Maverick MTX-242 

Max Puls:          2,10ms
Mitte mit Offset:  1,41ms
Mitte ohne Offset: 1,49ms
Min Puls:          0,94ms 

Sender:    Tactic TTX300
Empfänger: Tactic TR326

Max Puls:          1,80ms
Mitte mit Offset:  
Mitte ohne Offset: 1,48ms
Min Puls:          0,96ms

Throttle Channel 2


The control channel from transmitter, knows three fixes portInputRegister
   |---------------------|------------|
max vorward            center       max reverse
*/

// LED Outputs
const byte LedLowBatt =  9;
const byte LedForward = 12;
const byte LedNeutral = 11;
const byte LedReverse = 10;

// PPM In-/Outputs
const byte PpmInputSteering  = 3;
const byte PpmInputThrottle  = 2;
const byte PpmOutputSteering = 5;
const byte PpmOutputThrottle = 4;

// AD Input Throttle
const byte AdThrottleMin    = A0;
const byte AdThrottleMax    = A1;

// AD Input Steering
const byte AdSteeringMax    = A4;
const byte AdSteeringCenter = A3;
const byte AdSteeringMin    = A2;

// AD Input Battery Voltage
const byte AdBatteryVoltage = A5;

// Some defines 
const byte Steering = 0;
const byte Throttle = 1;

const uint16_t MinServoPuls    =  800;
const uint16_t MaxServoPuls    = 2200;
const uint16_t CenterServoPuls = 1500;

const uint16_t SteeringNeutralPosition = 1500;
const uint16_t ThrottleNeutralPosition = 1500;
const uint16_t EpsilonNeutralPosition  =   50;  // this value plus and minus
const uint16_t ThrottleTippPosition    = 1700;  // to chnage the direktion of gear

const uint16_t ThrottleTippDirektionChange =  4;
const uint16_t ThrottleTippCenterChange    = 40;


enum{
	SteeringPositionMax = 0,
	SteeringPositionNull,
	SteeringPositionMin
	};

enum{
	PotiThrottleMax = 0,
	PotiThrottleMin,
	PotiSteeringMax,
	PotiSteeringCenter,
	PotiSteeringMin
};
//volatile uint16_t SteeringCounter = 0;
volatile uint16_t ThrottleCounter = 0;

//volatile uint16_t SteeringTime = 0;
volatile uint16_t ThrottleInputTime = 0;
uint16_t ThrottleOutputTime = 1500;

//volatile bool SteeringReady = false;
volatile bool ThrottleReady = false;
uint8_t FindCenterCount = 16;

uint32_t StartTimer;

const uint8_t PotiChannelCount = 5;

uint8_t PotiChannelArray[PotiChannelCount] = {AdThrottleMax, AdThrottleMin, AdSteeringMax, AdSteeringCenter, AdSteeringMin};
uint16_t PotiValue[PotiChannelCount] = {512, 512, 512, 512, 512};
uint8_t NextPoti = 0;

uint8_t SteeringPosition = SteeringPositionNull;
uint16_t SteeringOutputTime = SteeringNeutralPosition;
bool RecallTipp = false;
uint16_t TippCounter = 0;

//void ISR_Steering ( void );
void ISR_Throttle ( void );
bool CeckPpmRangeMinMAx( uint16_t Value, uint16_t Min, uint16_t Max);
bool CeckPpmRangeEpsilon( uint16_t Value, uint16_t CheckValue, uint16_t Epsilon);
void SetLED(uint8_t LedPos);

Servo SteeringServo;
Servo ThrottleServo;



void setup() {
  // Config Serial
	Serial.begin(19200);
	Serial.print("Serial Ready");

  //Config PINs digital
	pinMode(LedLowBatt, OUTPUT);
	pinMode(LedForward, OUTPUT);
	pinMode(LedNeutral, OUTPUT);
	pinMode(LedReverse, OUTPUT);

	digitalWrite(LedLowBatt, HIGH);
	digitalWrite(LedForward, HIGH);
	digitalWrite(LedNeutral, LOW);
	digitalWrite(LedReverse, HIGH);

  //Servo Outputs
  SteeringServo.attach(PpmOutputSteering);
	ThrottleServo.attach(PpmOutputThrottle);
 
	//Interrupt
  //attachInterrupt(digitalPinToInterrupt(PpmInputSteering), ISR_Steering, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PpmInputThrottle), ISR_Throttle, CHANGE);

  //Config PINs analog
	analogReference(DEFAULT);
	
}

void loop() {

	//Check if two Pulse are ok.
  if(ThrottleReady)
	{
		//Serial.print(".");

    if (ThrottleInputTime > ThrottleTippPosition)
    {
  		TippCounter++;
			if (TippCounter > ThrottleTippCenterChange)
			{
				SteeringPosition = SteeringPositionNull;
				SetLED(SteeringPosition);
			}
			RecallTipp = true;
    }
		else
		{
		  if(RecallTipp == true)
			{
				if ((TippCounter > ThrottleTippDirektionChange) && (TippCounter < ThrottleTippCenterChange))
				{
					if(SteeringPosition == SteeringPositionMax) SteeringPosition = SteeringPositionMin;
					else                                        SteeringPosition = SteeringPositionMax;
				}
				RecallTipp = false;
				TippCounter = 0;
				SetLED(SteeringPosition);
			}
			
		}

		switch ( SteeringPosition )
		{
			case SteeringPositionMax :
			SteeringOutputTime = MaxServoPuls - PotiValue[PotiSteeringMax];
			break;
			case SteeringPositionNull :
			SteeringOutputTime = CenterServoPuls +(PotiValue[PotiSteeringCenter] - 512);
			break;
			case SteeringPositionMin :
			SteeringOutputTime = MinServoPuls + PotiValue[PotiSteeringMin];
			break;
		}
		
		uint16_t LimitThrottleInputTime = ThrottleInputTime;
		if (LimitThrottleInputTime > ThrottleNeutralPosition) LimitThrottleInputTime = ThrottleNeutralPosition;
		
		Serial.print("Input: ");Serial.print(LimitThrottleInputTime);
		Serial.print("  PotiMin: ");Serial.print(PotiValue[PotiThrottleMin]);
		Serial.print("  PotiMax: ");Serial.print(PotiValue[PotiThrottleMax]);

    uint16_t EngineStopPuls = CenterServoPuls + (PotiValue[PotiThrottleMin]);  // 1800ms 
		uint16_t EngineFullPull = CenterServoPuls - (PotiValue[PotiThrottleMax]);  //  900ms

		Serial.print("  Stop: ");Serial.print(EngineStopPuls);
		Serial.print("  FP: ");Serial.print(EngineFullPull);
		
				
		ThrottleOutputTime = map(LimitThrottleInputTime, MinServoPuls, CenterServoPuls, EngineFullPull, EngineStopPuls);

		Serial.print("  Out: ");Serial.println(ThrottleOutputTime);
				
	  ThrottleServo.writeMicroseconds(ThrottleOutputTime);
		SteeringServo.writeMicroseconds(SteeringOutputTime);
		ThrottleReady = false;
	}  
	
	//Every Loop get one AD Channel
	PotiValue[NextPoti] = analogRead(PotiChannelArray[NextPoti]);
	NextPoti++;
	NextPoti %= PotiChannelCount;
	//Serial.print("+");

}

bool CeckPpmRangeMinMAx( uint16_t Value, uint16_t Min, uint16_t Max)
{
  if((Value >= Min) && (Value <= Max)) return true;
	return false;
}

bool CeckPpmRangeEpsilon( uint16_t Value, uint16_t CheckValue, uint16_t Epsilon)
{
  if((Value >= (CheckValue - Epsilon)) && (Value <= (CheckValue + Epsilon))) return true;
	return false;
}

void SetLED(uint8_t LedPos)
{
	digitalWrite(LedForward, HIGH);
	digitalWrite(LedNeutral, HIGH);
	digitalWrite(LedReverse, HIGH);

  switch (LedPos)
	{
	  case SteeringPositionMax :
		  digitalWrite(LedForward, LOW);
			break;
	  case SteeringPositionNull :
		  digitalWrite(LedNeutral, LOW);
		  break;
	  case SteeringPositionMin :
		  digitalWrite(LedReverse, LOW);
		  break;
	}
}

void ISR_Throttle ( void )
{
  static uint32_t ThrottleStartTime;
  
	if(digitalRead(PpmInputThrottle) == HIGH)
  {
	  ThrottleStartTime = micros();
  }
  else
  {
	  uint32_t TempTime = micros() - ThrottleStartTime;
		if (CeckPpmRangeMinMAx(TempTime, MinServoPuls, MaxServoPuls))
	  {
		  ThrottleInputTime = uint16_t(TempTime);
			ThrottleReady = true;
      ThrottleCounter++;
		}
  }
}

// void ISR_Steering ( void )
// {
// 	static uint32_t SteeringStartTime;
// 
// 	if(digitalRead(PpmInputSteering) == HIGH)
// 	{
// 		SteeringStartTime = micros();
// 	}
// 	else
// 	{
// 		uint32_t TempTime = micros() - SteeringStartTime;
// 		if (CeckPpmRange(TempTime, MinServoPuls, MaxServoPuls))
// 		{

// 			SteeringTime = uint16_t(TempTime);

// 			SteeringReady = true;

// 			SteeringCounter++;
// 		}

// 	}
// }
