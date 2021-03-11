#include <Arduino.h>

// Define motor driver pins
#define IN1_PIN 6  // PWMB
#define IN2_PIN 10 // DIRB  ---  right
#define IN4_PIN 9  // PWMA
#define IN3_PIN 5  // DIRA  ---  left

// Define Cytron Maker Line sensor connection pins
//   Arduino MEGA pin:        Sensor pin:
#define LN_SENS_PIN_RIGHTEDGE 22       // right edge sensor - Connected to D1 pin of the sensor
// #define LN_SENS_PIN_RIGHT 23       // right sensor - Connected to D2 pin of the sensor
#define LN_SENS_PIN_RIGHT 25       // right sensor - Connected to D2 pin of the sensor
#define LN_SENS_PIN_MIDDLE 24       // middle sensor - Connected to D3 pin of the sensor
// #define LN_SENS_PIN_LEFT 25       // left sensor Connected to D4 pin of the sensor
#define LN_SENS_PIN_LEFT 23       // left sensor Connected to D4 pin of the sensor
#define LN_SENS_PIN_LEFTEDGE 26       // left edge sensor - Connected to D5 pin of the sensor
#define LN_SENS_CALIB_PIN 27   // Connected to CAL pin of the sensor
#define LN_SENS_ANALOG_PIN A15 // Connected to AN pin of the sensor

void setup()
{
  Serial.begin(115200);
  pinMode(LN_SENS_PIN_RIGHTEDGE, INPUT);
  pinMode(LN_SENS_PIN_RIGHT, INPUT);
  pinMode(LN_SENS_PIN_MIDDLE, INPUT);
  pinMode(LN_SENS_PIN_LEFT, INPUT);
  pinMode(LN_SENS_PIN_LEFTEDGE, INPUT);
  pinMode(LN_SENS_CALIB_PIN, OUTPUT);
  pinMode(LN_SENS_ANALOG_PIN, INPUT);
}
void loop()
{
  
  digitalWrite (LN_SENS_CALIB_PIN,0);
  delay(10);
 digitalWrite (LN_SENS_CALIB_PIN,1);
  // left = digitalRead(LN_SENS_PIN_RIGHT);
  // mid = digitalRead(LN_SENS_PIN_MIDDLE);
  // right = digitalRead(LN_SENS_PIN_LEFT);

  // leftEdge  = digitalRead(LN_SENS_PIN_RIGHTEDGE);
  // rightEdge  = digitalRead(LN_SENS_PIN_LEFTEDGE);
  // Serial.println(digitalRead(LN_SENS_PIN_MIDDLE));
  Serial.println("jdofhgdoj dofgih[dsfgo d[foigh[dfgn");
delay(5000);
}