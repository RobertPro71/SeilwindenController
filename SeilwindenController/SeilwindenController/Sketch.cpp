/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>
/*End of auto generated code by Atmel studio */


//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio

enum PpmInputStatus {
	Init = 0,
	WaitingPuls,
	MeasurePuls
};


struct ReceiverPPM{
  unsigned long TimeStartPuls;
  unsigned long TimeEndPuls;
  unsigned long TimePuls;
	byte Pin; 
  PpmInputStatus Status;
}


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

//                         TimeStartPuls, TimeEndPuls, TimePuls,             Pin, Status
ReceiverPPM PpmInput[2] = {{           0,           0,     1500, PpmInputSteering, Init},
                           {           0,           0,     1500, PpmInputThrottle, Init}};

void setup() {
  // Config Serial
	Serial.begin(9600);

  //Config PINs digital
	pinMode(LedLowBatt, OUTPUT);
	pinMode(LedForward, OUTPUT);
	pinMode(LedNeutral, OUTPUT);
	pinMode(LedReverse, OUTPUT);

	pinMode(PpmInputSteering, INPUT_PULLUP);
	pinMode(PpmInputThrottle, INPUT_PULLUP);
	pinMode(PpmOutputSteering, OUTPUT);
	pinMode(PpmOutputThrottle, OUTPUT);

  //Config PINs analog
	analogReference(DEFAULT);

}

void loop() {
  //From Steering to Throttle
  for(byte Idx = Steering; Idx <= Throttle; Idx++)
  {
    HandlePpmInput( &PpmInput[Idx] );
	}
}

void HandlePpmInput( PpmInput *Input)
{
	  switch (Input->Status)
		{
		  case Init : if(digitalRead(Input->Pin == LOW) Input->Status = WaitingPuls;
			case WaitingPuls : ;
			case MeasurePuls : ;
		}

}