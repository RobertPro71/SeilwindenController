/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>
#include "Servo.h"
/*End of auto generated code by Atmel studio */


//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio


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

volatile uint32_t SteeringStartTime = 0;
volatile uint32_t ThrottleStartTime = 0;

volatile uint16_t SteeringTime = 0;
volatile uint16_t ThrottleTime = 0;

volatile bool SteeringReady = false;
volatile bool ThrottleReady = false;

bool SetOutput = false;
unsigned long NextPulseTime;

void ISR_Steering ( void );
void ISR_Throttle ( void );

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

  SteeringServo.attach(PpmInputSteering);
//	ThrottleServo.attach(PpmInputThrottle);

	pinMode(PpmOutputSteering, OUTPUT);
	pinMode(PpmOutputThrottle, OUTPUT);

  //Interrupt
  attachInterrupt(digitalPinToInterrupt(PpmInputSteering), ISR_Steering, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PpmInputThrottle), ISR_Throttle, CHANGE);

  //Config PINs analog
	analogReference(DEFAULT);
}

void loop() {

	//Check if two Pulse are ok.
  if(ThrottleReady)
	{
	  ThrottleServo.writeMicroseconds(ThrottleTime);
		ThrottleReady = false;
		Serial.print(".");
	}  
	
	if(SteeringReady)
	{
		SteeringServo.writeMicroseconds(SteeringTime);
		SteeringReady = false;
		Serial.print("-");
	}
}


void ISR_Steering ( void )
{
  if(digitalRead(PpmInputSteering) == HIGH)
	{
    SteeringStartTime = micros();
	}	
	else
	{
		uint32_t TempTime = micros() - SteeringStartTime;
		if ((TempTime > MinServoPuls) && (TempTime < MaxServoPuls))
		{
			SteeringTime = uint16_t(TempTime);
			SteeringReady = true;
		}
	}
}

void ISR_Throttle ( void )
{
  if(digitalRead(PpmInputThrottle) == HIGH)
  {
	  ThrottleStartTime = micros();
  }
  else
  {
	  uint32_t TempTime = micros() - ThrottleStartTime;
	  if ((TempTime > MinServoPuls) && (TempTime < MaxServoPuls))
	  {
		  ThrottleTime = uint16_t(TempTime);
			ThrottleReady = true;
	  }
  }
}