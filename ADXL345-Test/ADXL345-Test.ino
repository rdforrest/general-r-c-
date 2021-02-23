
// Very useful guide to ADXL345 for use as level sensor with Nano:
// https://www.best-microcontroller-projects.com/adxl345.html
// By John Main © best-microcontroller-projects.com
// This sketch outputs serial data as 2 parameters (roll and pitch)
// for display in processing code on PC.
// Outputs roll value to servo on D5

#include "math.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "ADXL345.h"
#include <Servo.h>

Servo myservo;  // create servo object to control a servo
//Servo servo1;
int pos; // Servo postion
#define LED_PIN LED_BUILTIN
// Macros to allow 0 ~ 180 mapped to -90 to 90

ADXL345 accel;

bool blinkState = false;

void setup() {

  Wire.begin();
  Serial.begin(9600);

  myservo.attach(5);  // attaches the servo on a pin  to the servo object. Using pin 4 gives jitter problems.

  accel.initialize();

  accel.setRate(ADXL345_RATE_100);  // This is default but shows the value.
  accel.setFullResolution(1); // 0 => 10 bit mode.
  accel.setLowPowerEnabled(0);
  accel.setRange(0); // 0 => 2g, 3 => 16g

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  float r, x, y, z;
  int16_t ax, ay, az;

  // Datasheet: OPERATION AT VOLTAGES OTHER THAN 2.5 V
  // 3v3 X,Y 25mg too high, z 20mg too low
  // 3V3 lsb value 265/g c  (g/265)=0.03698
  // 2V5 lsb value 256/g   (g/256)=0.03828 z axis unaffected by voltage supply.
#define ADXL345_LSBVAL_3V3 3.698E-3
#define ADXL345_LSBVAL_2V5 3.828E-3

  accel.getAcceleration(&ax, &ay, &az);
  x = ax * ADXL345_LSBVAL_3V3 - 25E-3;
  y = ay * ADXL345_LSBVAL_3V3 - 25E-3;
  z = az * ADXL345_LSBVAL_2V5 + 20e-3;

  r = sqrt(x * x + y * y + z * z);
  // Angle from x,y axis to gravity vector.
  int roll   = 180 / M_PI * ( M_PI / 2 - (acos(y / r) ) );
  int pitch  = 180 / M_PI * ( M_PI / 2 - (acos(x / r) ) );

  Serial.print(roll); Serial.print(' ');
  Serial.print(pitch); Serial.print(' ');
  Serial.print('\n');

  // servo.writes to (pos);
  pos = pitch + 90; // Adjust servo output
  Serial.print("Servo output""\t"); //tab
  Serial.print(pos);
  Serial.print('\n'); // New line
  myservo.write(pos);
  delay(15);       // waits for the servo to get there

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);
  delay(50);
}
