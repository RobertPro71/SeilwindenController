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

// AD rating
const uint16_t MinAdValue   =    0;
const uint16_t MaxAdValue   = 1023;

// Some defines 
const byte Steering = 0;
const byte Throttle = 1;

const uint16_t MinServoPulsLimit =  800;
const uint16_t MinServoPuls      = 1000;
const uint16_t MaxServoPulsLimit = 2200;
const uint16_t MaxServoPuls      = 2000;
const uint16_t CenterServoPuls   = 1500;

const uint16_t SteeringNeutralPosition = 1500;
const uint16_t ThrottleNeutralPosition = 1500;
const uint16_t EpsilonNeutralPosition  =   50;  // this value plus and minus
const uint16_t ThrottleTippPosition    = 1700;  // to chnage the direktion of gear

const uint16_t ThrottleTippDirektionChange =  4;
const uint16_t ThrottleTippCenterChange    = 40;

// gear
const uint16_t MaxForwardServoPuls = MaxServoPuls + 200;  
const uint16_t MinForwardServoPuls = MaxServoPuls - 500; 

const uint16_t MaxReverseServoPuls = MinServoPuls - 200;  
const uint16_t MinReverseServoPuls = MinServoPuls + 500; 

const uint16_t MaxNeutralServoPuls = CenterServoPuls + 300;  
const uint16_t MinNeutralServoPuls = CenterServoPuls - 300;  

enum{
	eGearBoxForward = 0,
	eGearBoxNeutral,
	eGearBoxReverse
	};

const uint8_t PotiChannelCount = 5;

enum{
	PotiGearboxForward = 0,
	PotiGearboxNeutral,
	PotiGearboxReverse,
	PotiThrottleMax,
	PotiThrottleMin
};

// connect channel to AD channel
uint8_t PotiChannelArray[PotiChannelCount] = {AdSteeringMax, AdSteeringCenter, AdSteeringMin, AdThrottleMax, AdThrottleMin};
// store value of AD
uint16_t PotiValue[PotiChannelCount] = {512, 512, 512, 512, 512};
uint8_t NextPoti = 0;


volatile uint16_t ThrottleInputTime = 0;  //is set from interrupt function
uint16_t ThrottleOutputTime = 1500;       //
volatile bool PulsReady = false;      //if true, read usebale puls
//uint8_t FindCenterCount = 16;             //read 16 times same value to set as center time
//uint32_t StartTimer;


uint8_t GearboxPosition     = eGearBoxNeutral;
uint16_t GearboxOutputTime = SteeringNeutralPosition;
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

	//New Pulse is received
	if(PulsReady)
	{
		//Tipp position is to change the direktion
		// short tipp = do nothing
		// normal tipp = change direktion
		// long tipp = neutral position

		if (ThrottleInputTime > ThrottleTippPosition)
		{
			TippCounter++; // Not fine but simple delay
			//Long tipp
			if (TippCounter > ThrottleTippCenterChange)
			{
				GearboxPosition = eGearBoxNeutral;
				SetLED(GearboxPosition);
			}
			RecallTipp = true;
		}
		else
		{
			if(RecallTipp == true)
			{
				// normal tipp
				if ((TippCounter > ThrottleTippDirektionChange) && (TippCounter < ThrottleTippCenterChange))
				{
					// change direktion
					if(GearboxPosition == eGearBoxForward) GearboxPosition = eGearBoxReverse;
					else                                   GearboxPosition = eGearBoxForward;
				}
				RecallTipp = false;
				TippCounter = 0;
				// set right led
				SetLED(GearboxPosition);
			}
			
		}

		//Move the gearbox
		switch ( GearboxPosition )
		{
			case eGearBoxForward :
				GearboxOutputTime = map(PotiValue[PotiGearboxReverse], MinAdValue, MaxAdValue, MinForwardServoPuls, MaxForwardServoPuls);
				break;
			case eGearBoxNeutral :{
				GearboxOutputTime = map(PotiValue[PotiGearboxNeutral], MinAdValue, MaxAdValue, MinNeutralServoPuls, MaxNeutralServoPuls);
				//don't touch other positions
				uint16_t MaxTime = map(PotiValue[PotiGearboxReverse], MinAdValue, MaxAdValue, MinForwardServoPuls, MaxForwardServoPuls);
				uint16_t MinTime = map(PotiValue[PotiGearboxForward], MinAdValue, MaxAdValue, MinReverseServoPuls, MaxReverseServoPuls);
				GearboxOutputTime = constrain(GearboxOutputTime, MinTime, MaxTime);
			}
				break;
			case eGearBoxReverse :
				GearboxOutputTime = map(PotiValue[PotiGearboxForward], MinAdValue, MaxAdValue, MinReverseServoPuls, MaxReverseServoPuls);
				break;
		}
		
		uint16_t LimitThrottleInputTime = ThrottleInputTime;
		//Stick is push backwards, the value is not allowed for Throttle. So limit it
		LimitThrottleInputTime = min(LimitThrottleInputTime, ThrottleNeutralPosition);
		
		//Serial.print("Input: ");Serial.print(LimitThrottleInputTime);
		//Serial.print("  PotiMin: ");Serial.print(PotiValue[PotiGearboxNeutral]);
		//Serial.print("  PotiMax: ");Serial.print(PotiValue[PotiGearboxForward]);

		uint16_t EngineFullPuls = map(PotiValue[PotiThrottleMin], MinAdValue, MaxAdValue, MaxServoPuls - 400, MaxServoPuls + 200);
		uint16_t EngineStopPuls = map(PotiValue[PotiThrottleMax], MinAdValue, MaxAdValue, MinServoPuls - 200, MaxServoPuls + 400);  

// 		Serial.print("  St: ");Serial.print(EngineStopPuls);
// 		Serial.print("  FP: ");Serial.print(EngineFullPuls);
						
		ThrottleOutputTime = map(LimitThrottleInputTime, MinServoPuls, CenterServoPuls, EngineFullPuls, EngineStopPuls);

//  		Serial.print("P1: ");Serial.print(PotiValue[PotiGearboxForward]);
//  		Serial.print(" 2: ");Serial.print(PotiValue[PotiGearboxNeutral]);
//  		Serial.print(" 3: ");Serial.print(PotiValue[PotiGearboxReverse]);
//  		Serial.print(" 4: ");Serial.print(PotiValue[PotiThrottleMax]);
//  		Serial.print(" 5: ");Serial.print(PotiValue[PotiThrottleMin]);
		

//  		Serial.print("  Th: ");Serial.print(ThrottleOutputTime);
//  		Serial.print("  In: ");Serial.print(LimitThrottleInputTime);
// 	    Serial.print("  GB: ");Serial.println(GearboxOutputTime);
//	    Serial.println();
 				
		ThrottleServo.writeMicroseconds(ThrottleOutputTime);
		SteeringServo.writeMicroseconds(GearboxOutputTime);
		PulsReady = false;
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
	  case eGearBoxForward :
		  digitalWrite(LedForward, LOW);
			break;
	  case eGearBoxNeutral :
		  digitalWrite(LedNeutral, LOW);
		  break;
	  case eGearBoxReverse :
		  digitalWrite(LedReverse, LOW);
		  break;
	}
}

void ISR_Throttle ( void )
{
  static uint32_t ThrottleStartTime;
  
	if(digitalRead(PpmInputThrottle) == HIGH) // Puls start
	{
		ThrottleStartTime = micros(); // Remembertime at start
	}
	else
	{
		uint32_t DeltaTime = micros() - ThrottleStartTime; // calculate length of pulse
		if (CeckPpmRangeMinMAx(DeltaTime, MinServoPulsLimit, MaxServoPulsLimit))  // check pulse length
		{
			ThrottleInputTime = uint16_t(DeltaTime); //Transmit to main loop
			PulsReady = true; //Mark a new value
		}
	}
}
