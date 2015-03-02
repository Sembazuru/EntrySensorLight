/*
 * Entry Sensor Light: with Sharp Distance Sensor and PWM Output
 * - Use a distance sensor (Sharp# GP2Y0A710K0F) to ramp (fade) a PWM output
 *    up when the sensed distance changes. Then after a time out, ramp the PWM
 *    output down at a (potentially) different rate.
 *
 * By Chris "Sembazuru" Elliott, SembazuruCDE (at) GMail.com
 * 2013/10/16
 *
 * To the extent possible under law, Chris "Sembazuru" Elliott has waived all
 *  copyright and related or neighboring rights to SharpDistanceSensorPWMout.ino
 *  See http://creativecommons.org/publicdomain/zero/1.0/ for details.
 *  (Attribution would be nice, but in no way compulsory.)
 *
 * Hardware configuration on Arduino UNO:
 * - Analog output of sensor to A0
 * - LED13 used to show trigger status
 * - PWM on pin 3 used for the ramping output (pin 4 used as ground reference for
 *    testing with 0.1" 5V LED)
 * - A5 pin used to disable serial diagnostics (A4 pin used as ground reference
 *    for testing with 0.1" shorting pins)
 */

// My addon to the non-template version 1.02. See http://playground.arduino.cc/Main/runningMedian for details and downloads.
#include <RunningMedianWithAverageParameter.h>

// Pair of pins to enable serial diagnostics. Short these two pins (or just the diagnosticsPin to ground) to silence serial diagnostics.
const byte diagnosticsPin = A5;
const byte diagnosticsGnd = A4;

// Constants and variables for data collection.
const byte sensorPin = A0;
unsigned int currentADC;
unsigned int currentMedian;
const byte medianArraySize = 19; // number of data points to collect to calculate the median (class range is 5-19)
const byte medianAverageSize = 10; // number of data points to average in the middle of the sorted median array
const unsigned long readingInterval = 17; // Sensor readings take 16.5ms +/- 3.7ms.
unsigned long readingStart = millis();
RunningMedian myMedian = RunningMedian(medianArraySize);

// Constants and variables for triggering.
unsigned int lastMedian;
const unsigned int triggerThreshold = 7; // +/- value for median deltas
const byte triggerLED = 13; // Indicator LED for trigger
const unsigned long triggerDelay = 10000; // How long the trigger will last (minimum) in milliseconds
unsigned long triggerStart; // To allow capturing the time at the start of a trigger
boolean triggered = false;

// Constants and variables for ramping the output up and down.
boolean ramping = false; // true == need to ramp, false == done ramping.
boolean rampDir = false; // true == up, false == down.
const byte rampPin = 3; // PWM pin to ramp up and down.
const byte rampGnd = 4; // Adjacent ground provided for use of 0.1" spacing 5V LED.
const int rampMin = 0;
const int rampMax = 255;
const byte rampStep = 4; // How much to change the rampValue by each iteration.
const byte rampUDRatio = 2; // Amount to divide the rampStep by when ramping down to change the ramp up to ramp down rate ratio.
const unsigned long rampTime = 3000; // How many milliseconds (base) to take for the ramp up.
const unsigned long rampStepInterval = rampTime / ((rampMax - rampMin) / rampStep); // Calculate how long between ramp steps. Do this calculation once here.
unsigned long rampStepStart = millis();
int rampValue = rampMin;

void setup()
{
  Serial.begin(115200);
  while (!Serial); // Wait for serial port to connect. Needed for Leonardo only.
  delay(1000); // Simply to allow time for the ERW versions of the IDE time to automagically open the Serial Monitor. 1 second chosen arbitrarily.

  // Setup pins for enabling/disabling serial diagnostics.
  pinMode(diagnosticsGnd, OUTPUT);
  digitalWrite(diagnosticsGnd, LOW);
  pinMode(diagnosticsPin, INPUT_PULLUP);

  // Setup pins for using the onboard LED to indicate triggering.
  pinMode(triggerLED, OUTPUT);
  digitalWrite(triggerLED, LOW);

  // Setup pins for the ramping (fading) PWM pin.
  pinMode(rampGnd, OUTPUT);
  digitalWrite(rampGnd, LOW);
  analogWrite(rampPin, rampValue);
}

void loop()
{
  if ((millis() - readingStart) > readingInterval)
  {
    getData();
    triggerCheck();
    if (digitalRead(diagnosticsPin))
    {
      diagnostics(readingStart);
    }
  }
  if (ramping)
  {
    if ((millis() - rampStepStart) > rampStepInterval)
    {
      rampValue = rampOutput(rampValue, rampStep, rampDir);
      if (digitalRead(diagnosticsPin))
      {
        diagnostics(rampStepStart);
      }
    }
  }
}

void getData()
{
  readingStart = millis();
  currentADC = analogRead(sensorPin);
  myMedian.add(currentADC);
  lastMedian = currentMedian;
  currentMedian = myMedian.getMedian();
}

void triggerCheck()
{
  // Trigger if the median drops by more than the threshold or raises by more than the threshold.
  if ((currentMedian < (lastMedian - triggerThreshold)) || (currentMedian > (lastMedian + triggerThreshold)))
  {
    triggerStart = millis();
    triggered = true;
    ramping = true; // Start (or continue) ramping
    rampDir = true; //  up
    digitalWrite(triggerLED, HIGH);
  }
  // If currently triggered, check to see if enough time has passed since the last trigger event to turn off the trigger.
  if (triggered)
  {
    if ((millis() - triggerStart) > triggerDelay)
    {
      triggered = false;
      ramping =  true; // Start (or continue) ramping
      rampDir = false; //  down
      digitalWrite(triggerLED, LOW);
    }
  }
}

int rampOutput(int _value, int _step, boolean _dir)
{
  rampStepStart = millis();
  if (!_dir)
  {
    _step = -_step / rampUDRatio;
  }
  _value += _step;
  if (_value < rampMin)
  {
    _value = rampMin;
    ramping = false;
  }
  if (_value > rampMax)
  {
    _value = rampMax;
    ramping = false;
  }
  analogWrite(rampPin, _value);
  return _value;
}

void diagnostics(unsigned long _time)
{
  char _comma[] = ",";
  if (!lastMedian) // lastMedian should only ever be zero when the program starts.
  {
    // Use the F() macro throughout to not waste valueable SRAM on diagnostic messages.
    Serial.print(F("\"Time\""));
    Serial.print(_comma);
    Serial.print(F("\"myMedian.getCount()\""));
    Serial.print(_comma);
    Serial.print(F("\"currentADC\""));
    Serial.print(_comma);
    Serial.print(F("\"currentMedian\""));
    Serial.print(_comma);
    Serial.print(F("\"lastMedian\""));
    Serial.print(_comma);
    Serial.print(F("\"deltaMedian\"")); // Casting to signed values to allow a negative result.
    Serial.print(_comma);
    Serial.print(F("\"triggered\""));
    Serial.print(_comma);
    Serial.print(F("\"ramping\""));
    Serial.print(_comma);
    Serial.print(F("\"rampDir\""));
    Serial.print(_comma);
    Serial.print(F("\"rampValue\""));
    Serial.println();
  }
  Serial.print(_time);
  Serial.print(_comma);
  Serial.print(myMedian.getCount());
  Serial.print(_comma);
  Serial.print(currentADC);
  Serial.print(_comma);
  Serial.print(currentMedian);
  Serial.print(_comma);
  Serial.print(lastMedian);
  Serial.print(_comma);
  Serial.print(((int) currentMedian) - ((int) lastMedian)); // Casting to signed values to allow a negative result.
  Serial.print(_comma);
  Serial.print(triggered);
  Serial.print(_comma);
  Serial.print(ramping);
  Serial.print(_comma);
  Serial.print(rampDir);
  Serial.print(_comma);
  Serial.print(rampValue);
  Serial.println();
}


/*
void diagnostics(unsigned long _time)
 {
 // Use the F() macro throughout to not waste valueable SRAM on diagnostic messages.
 Serial.print(_time);
 Serial.print(F(" :"));
 Serial.print(F(" Size= "));
 Serial.print(myMedian.getCount());
 Serial.print(F(" Raw= "));
 Serial.print(currentADC);
 Serial.print(F(" Median= "));
 Serial.print(currentMedian);
 Serial.print(F(" Last= "));
 Serial.print(lastMedian);
 Serial.print(F(" Delta= "));
 Serial.print(((int) currentMedian) - ((int) lastMedian)); // Casting to signed values to allow a negative result.
 Serial.print(F(" Trigger= "));
 Serial.print(triggered);
 Serial.print(F(" Ramping= "));
 Serial.print(ramping);
 Serial.print(F(" Direction= "));
 Serial.print(rampDir);
 if (rampDir)
 {
 Serial.print(F("  UP "));
 }
 else
 {
 Serial.print(F(" DOWN"));
 }
 Serial.print(F(" PWM= "));
 Serial.print(rampValue);
 Serial.println();
 }
 */


