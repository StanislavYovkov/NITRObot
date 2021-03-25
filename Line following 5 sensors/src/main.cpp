//Line_following_5_sensors
#include <Arduino.h>

// The rangefinders work well to show the distance to objects from around
// 1 inch (2 cm) to around 9 feet away (3 meters), but they have trouble when
// they aren't approximately at a right angle to the object they are detecting.
// If the angle is too great (over about 15 degrees) not enough of the sound
// bounces back for it to get a reliable range.

//#include <Servo.h>

#define LEFT_FOR 9    // PWMB
#define LEFT_BACK 5   // DIRB  ---  Left
#define RIGHT_FOR 6   // PWMA
#define RIGHT_BACK 10 // DIRA  ---  Right

#define LN_SENS_PIN_RIGHTEDGE 22 // right edge sensor - Connected to D1 pin of the sensor
// #define LN_SENS_PIN_RIGHT 23       // right sensor - Connected to D2 pin of the sensor
#define LN_SENS_PIN_RIGHT 25  // right sensor - Connected to D2 pin of the sensor
#define LN_SENS_PIN_MIDDLE 24 // middle sensor - Connected to D3 pin of the sensor
// #define LN_SENS_PIN_LEFT 25       // left sensor Connected to D4 pin of the sensor
#define LN_SENS_PIN_LEFT 23     // left sensor Connected to D4 pin of the sensor
#define LN_SENS_PIN_LEFTEDGE 26 // left edge sensor - Connected to D5 pin of the sensor
#define LN_SENS_CALIB_PIN 27    // Connected to CAL pin of the sensor
#define LN_SENS_ANALOG_PIN A15  // Connected to AN pin of the sensor

const int LeftIrAvoidancePin = 12;
const int RightIrAvoidancePin = A5;
const int UltrasonicPin = 3;
const int RgbPin = 2;
const int ServoPin = 13;
const int LedPin = 33;

// Robot parameters:
// Robot length measured on the robot is 25.0 cm.
// Robot width measured on the robot is  16.7 cm.

// Maze parameters:
// In order for the robot to be able to safely make an U turn,
// we will choose the maze width to be 3 times the robot width,
// which is equal to 50.1, we will approximate this value to 50 cm.
const int MazeCorridorWidth = 50;

// Tresholds:
const float FrontDistanceTreshold = MazeCorridorWidth / 2;
const float WallToCorridorMiddle = MazeCorridorWidth / 2;
const float SideCorridorTreshold = MazeCorridorWidth;

const float CenterLineTolerance = 2.5; // plus/minus how many cm are acceptable to consider the movement to be on the center line...
                                       // +- 1 cm from centerline is considered straight movement!!!
const float SharpTurnTreshold = 15.0;  // Measured by experiments with the robot
const int WallFollowingSide = -90;     //Set: -90 for right wall following or +90 for left wall following
                                       //we will add this value to the servo position i.e. myservo.write(90 + WallFollowingSide);
                                       // in order to set to which side the servo should move (0 or 180 degrees)
//Servo parameters
const int FrontServoAngle = 90;
const int SideServoAngle = FrontServoAngle + WallFollowingSide; //(0 or 180 degrees)
const int FrontServoDelay = 150;
const int SideServoDelay = 150;

const int LeftSpeed = 95;
const int RightSpeed = 95;

float maxDistance = 130.0;
int speedLeft = LeftSpeed;
int speedRight = RightSpeed;
bool directionCompensation = false;

//Servo myservo;

void moveForward();
void moveBackward();
void turnLeft();
void turnRight();
void stopMoving();
float getDistance(int servoAngle, int delayAfterServoMovement); //read the Ultasonic Sensor pointing at the given servo angle

int left();
int mid();
int right();
int leftEdge();
int rightEdge();

//-----------------------------------------------

void setup()
{
  pinMode(LN_SENS_PIN_RIGHTEDGE, INPUT);
  pinMode(LN_SENS_PIN_RIGHT, INPUT);
  pinMode(LN_SENS_PIN_MIDDLE, INPUT);
  pinMode(LN_SENS_PIN_LEFT, INPUT);
  pinMode(LN_SENS_PIN_LEFTEDGE, INPUT);
  // pinMode(LN_SENS_CALIB_PIN, OUTPUT);
  pinMode(LN_SENS_ANALOG_PIN, INPUT);

  pinMode(ServoPin, OUTPUT);
  pinMode(LEFT_FOR, OUTPUT);
  pinMode(LEFT_BACK, OUTPUT);
  pinMode(RIGHT_FOR, OUTPUT);
  pinMode(RIGHT_BACK, OUTPUT);
  pinMode(UltrasonicPin, OUTPUT);
  pinMode(LedPin, OUTPUT);
  Serial.begin(9600);
  // myservo.attach(ServoPin);
  //  myservo.write(90); //Move the servo to center position

  // moveForward();
  delay(500);
}

//---------------------------------------------------------

void loop()
{

  // float frontDistance, sideDistance;

  int currentState = 0;
  // sideDistance = getDistance(SideServoAngle, SideServoDelay);
  //frontDistance = getDistance(FrontServoAngle, FrontServoDelay);

  /*
  Serial.print("LeftEdge: ");
  Serial.print(leftEdge);
  Serial.print(" Left: ");
  Serial.print(left);
  Serial.print(" Middle: ");
  Serial.print(mid);
  Serial.print(" Right: ");
  Serial.print(right);
  Serial.print(" RightEdge: ");
  Serial.println(rightEdge);

*/
  /*  if (frontDistance <= 15.0) //Стената отпред е близко
  {
    digitalWrite(LedPin, HIGH);
    currentState = 1;
  }
  if (frontDistance >= 25.0) //Стената отпред е далече
  {
    if (sideDistance >= 50.0) //Стената отдясно е далече
    {
      currentState = 2;
    }
    else if (sideDistance < 35.0 && sideDistance >= 29.0)     //    |_________|__ROBOT__|_________|_________|_________|     The robot is to the left from the centerline treshold
    {                                                         //    50cm      35cm      29cm      21cm      15cm      0cm
      currentState = 3;
    }
    else if (sideDistance > 15.0 && sideDistance <= 21.0)     //    |_________|_________|_________|__ROBOT__|_________|          The robot is to the right from the centerline treshold
    {                                                         //    50cm      35cm      29cm      21cm      15cm      0cm
      currentState = 4;
    }
    else if (sideDistance <= 15.0)                            //    |_________|_________|_________|_________|__ROBOT__|           The robot is on the far right os the corridor
    {                                                         //    50cm      35cm      29cm      21cm      15cm      0cm
      currentState = 5;
    }
    else if (sideDistance >= 35.0 && sideDistance < 50.0)     //    |__ROBOT__|_________|_________|_________|_________|     The robot is on the far left os the corridor
    {                                                         //    50cm      35cm      29cm      21cm      15cm      0cm
      currentState = 6;
    }
    else if (sideDistance >= 21.0 && sideDistance < 29.0)     //    |_________|_________|__ROBOT__|_________|_________|   The robot is close to the center line
    {                                                         //    50cm      35cm      29cm      21cm      15cm      0cm
      currentState = 7;
    } 
  }   
  switch (currentState)
  {
  case 1: // Turn 90 degrees left
    moveBackward();
    delay(100);
    speedLeft = LeftSpeed * 1.35;
    turnLeft();
    delay(750);
    moveBackward();
    delay(100);
    break;
  case 2: // Turn 90 degrees right
    for (size_t i = 0; i < 8; i++)
    {
      speedLeft = LeftSpeed;
      speedRight = RightSpeed;
      moveForward();
      delay(20);
      speedLeft = LeftSpeed * 1.4;
      speedRight = 0;
      moveForward();
      delay(170);
    }
    for (size_t i = 0; i < 7; i++)
    {
      speedLeft = LeftSpeed;
      speedRight = RightSpeed;
      moveForward();
      delay(35);
      stopMoving();
      delay(20);
    }
    moveBackward();
    delay(100);
    break;
  case 3: // Turn slight right
    speedRight = RightSpeed * 2.55;
    turnRight();
    delay(50);
    break;
  case 4: // Turn slight left
    speedLeft = LeftSpeed * 2.55;
    turnLeft();
    delay(50);
    break;
  case 5: // Turn  more agressive to the left
    speedLeft = LeftSpeed * 2.55;
    turnLeft();
    delay(100);
    break;
  case 6: // Turn  more agressive to the right
    speedRight = RightSpeed * 2.55;
    turnRight();
    delay(100);
    break;
  case 7: // Go forward
  // Nothing to do here, continue with the execution of the loop
    break;
  default:
    break;
  }  */
  //QQQQQQQQQQQqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqqq

  if ((leftEdge() == 1) && (left() == 1) && (mid() == 0) && (right() == 1) && (rightEdge() == 1)) //                1 1 0 1 1
  {
    currentState = 1;
  }
  else if ((leftEdge() == 1) && (left() == 0) && (mid() == 0) && (right() == 1) && (rightEdge() == 1)) //           1 0 0 1 1
  {
    currentState = 2;
  }
  else if ((leftEdge() == 1) && (left() == 0) && (mid() == 1) && (right() == 1) && (rightEdge() == 1)) //            1 0 1 1 1
  {
    currentState = 3;
  }
  else if ((leftEdge() == 0) && (left() == 0) && (mid() == 1) && (right() == 1) && (rightEdge() == 1)) //            0 0 1 1 1
  {
    currentState = 4;
  }
  else if ((leftEdge() == 0) && (left() == 1) && (mid() == 1) && (right() == 1) && (rightEdge() == 1)) //            0 1 1 1 1
  {
    currentState = 5;
  }
  else if ((leftEdge() == 1) && (left() == 1) && (mid() == 0) && (right() == 0) && (rightEdge() == 1)) //            1 1 0 0 1
  {
    currentState = 6;
  }
  else if ((leftEdge() == 1) && (left() == 1) && (mid() == 1) && (right() == 0) && (rightEdge() == 1)) //            1 1 1 0 1
  {
    currentState = 7;
  }
  else if ((leftEdge() == 1) && (left() == 1) && (mid() == 1) && (right() == 0) && (rightEdge() == 0)) //            1 1 1 0 0
  {
    currentState = 8;
  }
  else if ((leftEdge() == 1) && (left() == 1) && (mid() == 1) && (right() == 1) && (rightEdge() == 0)) //            1 1 1 1 0
  {
    currentState = 9;
  }
  else if ((leftEdge() == 0) && (left() == 0) && (mid() == 0) && (right() == 0) && (rightEdge() == 0)) //            0 0 0 0 0
  {
    currentState = 10;
  }

  switch (currentState)
  {
  case 1:
    speedLeft = LeftSpeed;
    speedRight = RightSpeed;
    moveForward();
    Serial.println("case1");
    break;
  case 2:
    speedLeft = LeftSpeed * 0.4;
    speedRight = RightSpeed * 1.0;
    moveForward();
    Serial.println("case2");
    break;
  case 3:
    speedLeft = LeftSpeed * 0.3;
    speedRight = RightSpeed * 1.6;
    moveForward();
    Serial.println("case3");
    break;
  case 4:
    speedLeft = LeftSpeed * 0.2;
    speedRight = RightSpeed * 2.0;
    moveForward();
  case 5:
    speedLeft = LeftSpeed * 0.1;
    speedRight = RightSpeed * 2.6;
    moveForward();
    Serial.println("case5");
    break;
  case 6:
    speedLeft = LeftSpeed * 1.0;
    speedRight = RightSpeed * 0.4;
    moveForward();
    Serial.println("case7");
    break;
  case 7:
    speedLeft = LeftSpeed * 1.6;
    speedRight = RightSpeed * 0.3;
    moveForward();
    Serial.println("case8");
    break;
  case 8:
    speedLeft = LeftSpeed * 2.0;
    speedRight = RightSpeed * 0.2;
    moveForward();
    Serial.println("case9");
    break;
  case 9:
    speedLeft = LeftSpeed * 2.6;
    speedRight = RightSpeed * 0.1;
    moveForward();
    Serial.println("case10");
    break;
  case 10:
    stopMoving();
    delay(1000);
    break;
  default:
    break;
  }
}

//==================================== FUNCTIONS =====================================================

void moveForward() // Move forward
{
  analogWrite(LEFT_FOR, abs(speedLeft));
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, abs(speedRight));
  analogWrite(RIGHT_BACK, LOW);
}

void moveBackward() // Move backward
{
  analogWrite(LEFT_FOR, LOW);
  analogWrite(LEFT_BACK, abs(speedLeft));
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, abs(speedRight));
}

void turnLeft() // Turn Left
{
  analogWrite(LEFT_FOR, LOW);
  analogWrite(LEFT_BACK, speedLeft);
  analogWrite(RIGHT_FOR, speedLeft);
  analogWrite(RIGHT_BACK, LOW);
}

void turnRight() // Turn Right
{
  analogWrite(LEFT_FOR, speedRight);
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, speedRight);
}

void stopMoving() // Stop movement
{
  analogWrite(LEFT_FOR, HIGH);
  analogWrite(LEFT_BACK, HIGH);
  analogWrite(RIGHT_FOR, HIGH);
  analogWrite(RIGHT_BACK, HIGH);
}
/*
float getDistance(int servoAngle, int delayAfterServoMovement)
{
  float distance;
  myservo.write(servoAngle);
  delay(FrontServoDelay);
  /*---------------------------150 millis
  speedLeft = LeftSpeed;
  speedRight = RightSpeed;
  moveForward();
  delay(35);
  stopMoving();
  delay(20);
  moveForward();
  delay(35);
  stopMoving();
  delay(20);
  moveForward();
  delay(40);
  stopMoving();
  -----------------------
  pinMode(UltrasonicPin, OUTPUT);
  digitalWrite(UltrasonicPin, LOW);
  delayMicroseconds(2);
  digitalWrite(UltrasonicPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(UltrasonicPin, LOW);
  pinMode(UltrasonicPin, INPUT);
  distance = pulseIn(UltrasonicPin, HIGH) / 58.00;
  return distance;
}*/

int left()
{
  int distance;
  distance = digitalRead(LN_SENS_PIN_RIGHT);
  return distance;
}

int mid()
{
  int distance;
  distance = digitalRead(LN_SENS_PIN_MIDDLE);
  return distance;
}

int right()
{
  int distance;
  distance = digitalRead(LN_SENS_PIN_LEFT);
  return distance;
}

int leftEdge()
{
  int distance;
  distance = digitalRead(LN_SENS_PIN_RIGHTEDGE);
  return distance;
}

int rightEdge()
{
  int distance;
  distance = digitalRead(LN_SENS_PIN_LEFTEDGE);
  return distance;
}