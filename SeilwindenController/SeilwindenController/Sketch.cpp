/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>
#include "Servo.h"
/*End of auto generated code by Atmel studio */


//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio

/*
Sender:    Maverick MTX-242
Empfänger: Maverick MTX-242 

Max Puls:          2,10ms
Mitte mit Offset:  1,41ms
Mitte ohne Offset: 1,49ms
Min Puls:          0,94ms 

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
const byte AdThrottleMax    = A0;
const byte AdThrottleMin    = A1;

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

const uint16_t FindNeutralTimer = 500;

const uint16_t SteeringNeutralPosition = 1500;
const uint16_t ThrottleNeutralPosition = 1700;
const uint16_t EpsilonNeutralPosition  =  200;  // this value plus and minus

volatile uint16_t SteeringCounter = 0;
volatile uint16_t ThrottleCounter = 0;

volatile uint16_t SteeringTime = 0;
volatile uint16_t ThrottleTime = 0;

volatile bool SteeringReady = false;
volatile bool ThrottleReady = false;

uint32_t StartTimer;

const uint8_t AdChannelCount = 5;

uint8_t AdChannelArray[AdChannelCount] = {AdThrottleMax, AdThrottleMin, AdSteeringMax, AdSteeringCenter, AdSteeringMin};
uint16_t AdValueArray[AdChannelCount] = {0, 0, 0, 0, 0};
uint8_t NextAd = 0;

void ISR_Steering ( void );
void ISR_Throttle ( void );
bool CeckPpmRange( uint16_t Value, uint16_t Min, uint16_t Max);
bool CeckPpmRange( uint16_t Value, uint16_t CheckValue, uint16_t Epsilon);

Servo SteeringServo;
Servo ThrottleServo;

void setup() {
  // Config Serial
	Serial.begin(9600);
	Serial.print("Serial Ready");

  //Config PINs digital
	pinMode(LedLowBatt, OUTPUT);
	pinMode(LedForward, OUTPUT);
	pinMode(LedNeutral, OUTPUT);
	pinMode(LedReverse, OUTPUT);

	digitalWrite(LedLowBatt, HIGH);
	digitalWrite(LedForward, HIGH);
	digitalWrite(LedNeutral, HIGH);
	digitalWrite(LedReverse, HIGH);

  //Servo Outputs
  SteeringServo.attach(PpmOutputSteering);
	ThrottleServo.attach(PpmOutputThrottle);
 
	//Interrupt
  attachInterrupt(digitalPinToInterrupt(PpmInputSteering), ISR_Steering, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PpmInputThrottle), ISR_Throttle, CHANGE);

  //Config PINs analog
	analogReference(DEFAULT);
	
	bool FindNetral = false;
	
// 	while(FindNetral == false)
// 	{
// 		if((SteeringTime < (SteeringNeutralPosition - EpsilonNeutralPosition)) &&	(SteeringTime > (SteeringNeutralPosition + EpsilonNeutralPosition)))
// 		  
// 	}
	
	
}

void loop() {

	//Check if two Pulse are ok.
  if(ThrottleReady && SteeringReady)
	{
	  ThrottleServo.writeMicroseconds(ThrottleTime);
		SteeringServo.writeMicroseconds(SteeringTime);
		ThrottleReady = false;
		SteeringReady = false;
	}  
	
	//Every Loop get one AD Channel
	AdValueArray[NextAd] = analogRead(AdArray[NextAd]);
	NextAd++;
	NextAd %= AdChannelCount;
	
	
		
}

bool CeckPpmRange( uint16_t Value, uint16_t Min, uint16_t Max)
{
  if((Value >= Min) && (Value <= Max)) return true;
	return false;
}

bool CeckPpmRange( uint16_t Value, uint16_t CheckValue, uint16_t Epsilon)
{
  if((Value >= (CheckValue - Epsilon)) && (Value <= (CheckValue + Epsilon))) return true;
	return false;
}


void ISR_Steering ( void )
{
  static uint32_t SteeringStartTime;

  if(digitalRead(PpmInputSteering) == HIGH)
	{
    SteeringStartTime = micros();
	}	
	else
	{
		uint32_t TempTime = micros() - SteeringStartTime;
		if (CeckPpmRange(TempTime, MinServoPuls, MaxServoPuls))
		{
			SteeringTime = uint16_t(TempTime);
			SteeringReady = true;
      SteeringCounter++;
		}
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
		if (CeckPpmRange(TempTime, MinServoPuls, MaxServoPuls))
	  {
		  ThrottleTime = uint16_t(TempTime);
			ThrottleReady = true;
      ThrottleCounter++;
		}
  }
}

