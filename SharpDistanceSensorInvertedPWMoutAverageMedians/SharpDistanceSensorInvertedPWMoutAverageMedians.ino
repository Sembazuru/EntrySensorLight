/*
 * Entry Sensor Light: with Sharp Distance Sensor and inverted PWM Output
 * - Use a distance sensor (Sharp# GP2Y0A710K0F) to fade a PWM output up when
 *    the sensed distance changes. Then after a time out, fade the PWM output
 *    down at a (potentially) different rate.
 *
 * By Chris "Sembazuru" Elliott, SembazuruCDE (at) GMail.com
 * 2014/05/13
 *
 * To the extent possible under law, Chris "Sembazuru" Elliott has waived all
 *  copyright and related or neighboring rights to SharpDistanceSensorPWMout.ino
 *  See http://creativecommons.org/publicdomain/zero/1.0/ for details.
 *  (Attribution would be nice, but in no way compulsory.)
 *
 * Hardware configuration on Arduino UNO:
 * - Analog output of sensor to A0
 * - LED13 used to show trigger status
 * - PWM on pin 3 used for the fading output (inverted so off is PWM255 and on is PWM0)
 *    This allows driving the base of a transistor
 * - A5 pin used to disable serial diagnostics (A4 pin used as ground reference
 *    for testing with 0.1" shorting pins). Default format is CSV with column
 *    header line every time diagnostics is started. 
 * - A3 pin used to enable UECIDE grapher plugin diagnostics (again, can use the
 *    A4 pin for ground reference since A5 is mutually exclusive). 
 */

// See http://playground.arduino.cc/Main/runningMedian for details and downloads.
#include <RunningMedian.h>

// Pair of pins to enable serial diagnostics.
// Short these two pins (or just the diagnosticsPin to ground) to silence serial diagnostics.
const byte diagnosticsPin = A5;
const byte diagnosticsGnd = A4;
boolean startDiagnostics = true;
// Pin, constants, and global variables for UECIDE Grapher formatted serial diagnostics.
const byte grapherPin = A3;
boolean startGrapher = true;

// Constants and variables for data collection.
const byte sensorPin = A0;
unsigned int currentADC;
float currentAverage;
const byte medianArraySize = 19; // number of data points to collect to calculate the average (class range is 5-19)
const byte medianAverageSize = 9; // number of data points to average in the middle of the sorted median array to get an average reading while ignoring outliers
const unsigned long readingInterval = 17; // Sensor readings take 16.5ms +/- 3.7ms.
unsigned long readingStart = millis();
RunningMedian myMedian = RunningMedian(medianArraySize);

// Constants and variables for triggering.
const byte PAcnt = 5; // highest index number for the array holding previous values.
//             The number of previous values will be this plus 2 because array indexes start counting at 0 not 1,
//             and comparison is based on the overflow value.
float previousAverages[PAcnt + 1]; // holding array for previous values to be used as a small pipe
float lastSignificantAverage = 0; // will recieve the previousAverages pipe overflow and be used in the trigger comparison
const unsigned int triggerThreshold = 5; // +/- value for average deltas to indicate if a trigger should happen
const byte triggerLED = 13; // Indicator LED for trigger
const unsigned long triggerDelay = 10000; // How long the trigger will last (minimum) in milliseconds
unsigned long triggerStart; // To allow capturing the time at the start of a trigger
boolean triggered = false; // flag for trigger state

// Constants and variables for fading the output up and down.
boolean fading = false; // true == need to fade, false == done fading.
boolean fadeDir = false; // true == fade towards on (towards 0), false == fade towards off (towards 255).
const byte fadePin = 3; // PWM pin to fade brighter and dimmer to drive a transistor base.
const int fadeMin = 0; // Inverted output makes this equivalent to LEDs on.
const int fadeMax = 255; // Inverted output makes this equivalent to LEDs off.
const char fadeStep = -3; // How much to change the fadeValue by each iteration. This should be negative for inverted operation.
const byte fadeUDRatio = 3; // Amount to divide the fadeStep by when fading down to change the fade up to fade down rate ratio.
const unsigned long fadeTime = 3000; // How many milliseconds (base) to take for the fade to on.
const unsigned long fadeStepInterval = fadeTime / ((fadeMax - fadeMin) / -fadeStep); // Calculate how long between fade steps. Do this calculation once here.
unsigned long fadeStepStart = millis();
int fadeValue = fadeMax;

void setup()
{
	Serial.begin(1152000);
	while (!Serial); // Wait for serial port to connect. Needed for Leonardo only.
	delay(1000); // Simply to allow time for the ERW versions of the IDE time to automagically open the Serial Monitor. 1 second chosen arbitrarily.

	// Setup pins for enabling/disabling serial diagnostics.
	pinMode(diagnosticsGnd, OUTPUT);
	digitalWrite(diagnosticsGnd, LOW);
	pinMode(diagnosticsPin, INPUT_PULLUP);
	pinMode(grapherPin, INPUT_PULLUP);

	// Setup pins for using the onboard LED to indicate triggering.
	pinMode(triggerLED, OUTPUT);
	digitalWrite(triggerLED, LOW);

	// Setup pins for the fading PWM pin.
	analogWrite(fadePin, fadeValue);

	// Initialize previousAvereages array here because it broke above.
	for (int i = 0; i < PAcnt; i++)
	{
		previousAverages[i] = 0; // Zero it out. Don't want undefined values mucking about.
	}
}

void loop()
{
	if ((millis() - readingStart) > readingInterval)
	{
		getData();
		triggerCheck();
		diagnostics(readingStart);
	}
	if (fading)
	{
		if ((millis() - fadeStepStart) > fadeStepInterval)
		{
			fadeValue = fadeOutput(fadeValue, fadeStep, fadeDir);
			diagnostics(fadeStepStart);
		}
	}
}

void getData()
{
	do
	{
		readingStart = millis();
		currentADC = analogRead(sensorPin);
		myMedian.add(currentADC);
		lastSignificantAverage = previousAverages[PAcnt]; // grab the last value as overflow
		for (int i = PAcnt; i > 0; i--)
		{
			previousAverages[i] = previousAverages[i - 1]; // shift all the values up one spot in the array
		}
		previousAverages[0] = currentAverage; // put the previous average in the beginning of the array
		currentAverage = myMedian.getAverage(medianAverageSize); // get the new average
	}
	while (myMedian.getCount() < myMedian.getSize()); // If the array isn't full loop back and take another reading.
}

void triggerCheck()
{
	// Trigger if the median drops by more than the threshold or raises by more than the threshold.
	if ((currentAverage < (lastSignificantAverage - triggerThreshold)) || (currentAverage > (lastSignificantAverage + triggerThreshold)))
	{
		triggerStart = millis();
		triggered = true;
		fading = true; // Start (or continue) fading...
		fadeDir = true; //  ...towards on
		digitalWrite(triggerLED, HIGH);
	}
	// If currently triggered, check to see if enough time has passed since the last trigger event to turn off the trigger.
	if (triggered)
	{
		if ((millis() - triggerStart) > triggerDelay)
		{
			triggered = false;
			fading =  true; // Start (or continue) fading...
			fadeDir = false; //  ...towards off
			digitalWrite(triggerLED, LOW);
		}
	}
}

int fadeOutput(int _value, int8_t _step, boolean _dir) // _value is an int to allow a negative 8-bit value
{
	fadeStepStart = millis();
	if (!_dir) // if fading towards off... (positve going _value)
	{
		_step = -_step / fadeUDRatio; // ...set step value positive and scale the rate
	}
	_value += _step;
	if (_value < fadeMin)
	{
		_value = fadeMin;
		fading = false;
	}
	if (_value > fadeMax)
	{
		_value = fadeMax;
		fading = false;
	}
	analogWrite(fadePin, _value);
	return _value;
}

void diagnostics(unsigned long _time)
{

	if (!digitalRead(diagnosticsPin)) // Check the diagnostic pin, only send out diagnostics if the pin is high.
	{
		startDiagnostics = true; // re-enable the header line for CSV output
		startGrapher = true; // re-enable the header line for UECIDE Grapher output
		return;
	}


	if (digitalRead(grapherPin)) // Check the UECIDE Grapher pin, send out CSV diagnostics if the pin is low.
	{
		if (startDiagnostics)
		{
			// Use the F() macro throughout to not waste valueable SRAM on diagnostic messages.
			startDiagnostics = false;
			startGrapher = true; // re-enable the header line for UECIDE Grapher output
			Serial.print(F("\"Time\""));
			Serial.print(',');
			Serial.print(F("\"myMedian.getCount()\""));
			Serial.print(',');
			Serial.print(F("\"currentADC\""));
			Serial.print(',');
			Serial.print(F("\"currentAverage\""));
			Serial.print(',');
			Serial.print(F("\"lastSignificantAverage\""));
			Serial.print(',');
			Serial.print(F("\"deltaAverage\""));
			Serial.print(',');
			Serial.print(F("\"triggered\""));
			Serial.print(',');
			Serial.print(F("\"fading\""));
			Serial.print(',');
			Serial.print(F("\"fadeDir\""));
			Serial.print(',');
			Serial.print(F("\"fadeValue\""));
			Serial.println();
		}
		Serial.print(_time);
		Serial.print(',');
		Serial.print(myMedian.getCount());
		Serial.print(',');
		Serial.print(currentADC);
		Serial.print(',');
		Serial.print(currentAverage);
		Serial.print(',');
		Serial.print(lastSignificantAverage);
		Serial.print(',');
		Serial.print(currentAverage - lastSignificantAverage);
		Serial.print(',');
		Serial.print(triggered);
		Serial.print(',');
		Serial.print(fading);
		Serial.print(',');
		Serial.print(fadeDir);
		Serial.print(',');
		Serial.print(fadeValue);
		Serial.println();
	}
	else // 
	{
		if (startGrapher)
		{
			startDiagnostics = true; // re-enable the header line for CSV output
			startGrapher = false;
			Serial.println(F("R\nR\nR")); // Reset graphing window "It is recommended to start your graph with a couple of these commands to get the serial system into sync."
			Serial.println(F("S1920:1024")); // Screen size x:y
			Serial.println(F("M10:10:60:20")); // Margins top:right:bottom:left
			Serial.println(F("B200:200:200")); // Background Color red:green:blue
			Serial.println(F("F0:0:0")); // Axis color red:green:blue
			Serial.println(F("Y0:1024:64")); // Y axis scale min:max:step

			Serial.println(F("AcurrentADC:0:255:0")); // Data set and color title:r:g:b
			Serial.println(F("AcurrentAverage:0:0:255"));
			Serial.println(F("AlastSignificantAverage:255:255:0"));
			Serial.println(F("AdeltaAverage:255:0:0"));
			Serial.println(F("Atriggered:255:0:255"));
			Serial.println(F("Afading:128:0:0"));
			Serial.println(F("AfadeDir:0:128:0"));
			Serial.println(F("AfadeValue:0:0:128"));
		}
		Serial.print('V'); // Add data point values a:b:c:...:n
		Serial.print(currentADC);
		Serial.print(':');
		Serial.print(currentAverage);
		Serial.print(':');
		Serial.print(lastSignificantAverage);
		Serial.print(':');
		Serial.print(currentAverage - lastSignificantAverage);
		Serial.print(':');
		Serial.print(triggered);
		Serial.print(':');
		Serial.print(fading);
		Serial.print(':');
		Serial.print(fadeDir);
		Serial.print(':');
		Serial.println(fadeValue);
	}
}
