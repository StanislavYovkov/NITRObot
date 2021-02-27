#include <Arduino.h>

// The rangefinders work well to show the distance to objects from around
// 1 inch (2 cm) to around 9 feet away (3 meters), but they have trouble when
// they aren't approximatley at a right angle to the object they are detecting.
// If the angle is too great (over about 15 degrees) not enough of the sound
// bounces back for it to get a reliable range.

// Тук е секцията за Библиотеки и хедър файлове които се инклудват:

#include <Servo.h>

// PROBEN TEKST

// Тук е секцията с описание на използваните пинове:

// дефинициите са изцяло с главни букви
#define LEFT_FOR 9    // PWMB
#define LEFT_BACK 5   // DIRB  ---  Left
#define RIGHT_FOR 6   // PWMA
#define RIGHT_BACK 10 // DIRA  ---  Right

const int LeftIrAvoidancePin = 12;
const int RightIrAvoidancePin = A5;
const int UltrasonicPin = 3;
const int RgbPin = 2;
const int ServoPin = 13;
const int LedPin = 33;

// Robot parameters:
const int RobotLenght = 25;     // реални размери 25.0 см.
const int robotWidth = 16;      // реални размери 16.7 см.
const int brakingDistance = 10; // спирачен път в см.
// Maze parameters:
const int MazeCorridorWidth = 50; //   ШИРОЧИНАТА НА КОРИДОРЕ  Е 3 ШИРИНИ НА РОБОТА = 50см. (robotWidth * 3) + 2;

// Tresholds:
const float FrontDistanceTreshold = MazeCorridorWidth / 2 + brakingDistance;
const float WallToCorridorMiddle = MazeCorridorWidth / 2;
const float SideCorridorTreshold = MazeCorridorWidth;

//? С теаи параметри правим нещо като ръчен PID алгоритъм за следване на централната линия спрямо измереното разстояние до страничната стена
const float CenterLineTolerance = 2.5; // plus/minus how many cm are acceptable to consider the movement to be on the center line...
                                       // +- 1 cm from centerline is considered straight movement!!!
const float SharpTurnTreshold = 15.0;  // TODO Да се определи спрямо размера робота и коридора !!!!!

//? От тук задаваме дали ще следваме дясна или лява стена за да е универсале алгоритъма
const int WallFollowingSide = -90; //Set: -90 for right wall following or +90 for left wall following
                                   //we will add this value to the servo position i.e. myservo.write(90 + WallFollowingSide);
                                   // in order to set to which side the servo should move (0 or 180 degrees)
//Servo parameters
const int FrontServoAngle = 90;
const int SideServoAngle = FrontServoAngle + WallFollowingSide; //(0 or 180 degrees)
const int FrontServoDelay = 150;
const int SideServoDelay = 150;

const int LeftSpeed = 90;
const int RightSpeed = 90;

float maxDistance = 130.0;
int speedLeft = LeftSpeed;
int speedRight = RightSpeed;
bool directionCompensation = false;

Servo myservo;

void moveForward();
void moveBackward();
void customTurnLeft();
void customTurnRight();
void stopMoving();
float getDistance(int servoAngle, int delayAfterServoMovement); //read the Ultasonic Sensor pointing at the given servo angle

//-----------------------------------------------

void setup()
{
  pinMode(ServoPin, OUTPUT);
  pinMode(LEFT_FOR, OUTPUT);
  pinMode(LEFT_BACK, OUTPUT);
  pinMode(RIGHT_FOR, OUTPUT);
  pinMode(RIGHT_BACK, OUTPUT);
  pinMode(UltrasonicPin, OUTPUT);
  pinMode(LedPin, OUTPUT);
  Serial.begin(9600);
  myservo.attach(ServoPin);
  myservo.write(90); //Move the servo to center position

  moveForward();
  delay(500);
}

//---------------------------------------------------------

void loop()
{

  float frontDistance, sideDistance;
  
  int currentState = 0; 
  sideDistance = getDistance(SideServoAngle, SideServoDelay);
  frontDistance = getDistance(FrontServoAngle, FrontServoDelay);

  if (frontDistance <= 15.0) //Стената отпред е близко 
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
    else if (sideDistance < 35.0 && sideDistance >= 29.0) //    |_________|__ROBOT__|_________|_________|_________|    - робота се намита тук!
    {                                                     //    50sm      35sm      29sm      21sm      15sm      0sm
      currentState = 3;                                   
    }
    else if (sideDistance > 15.0 && sideDistance <= 21.0) //    |_________|_________|_________|__ROBOT__|_________|    - робота се намита тук!
    {                                                     //    50sm      35sm      29sm      21sm      15sm      0sm
      currentState = 4; 
    }
    else if (sideDistance <= 15.0)                        //    |_________|_________|_________|_________|__ROBOT__|    - робота се намита тук!
    {                                                     //    50sm      35sm      29sm      21sm      15sm      0sm
      currentState = 5;
    }
    else if (sideDistance >= 35.0 && sideDistance < 50.0) //    |__ROBOT__|_________|_________|_________|_________|    - робота се намита тук!
    {                                                     //    50sm      35sm      29sm      21sm      15sm      0sm
      currentState = 6;
    }// ако не са изпълнени 4-те условия робота cе намира тук   |_________|_________|__ROBOT__|_________|_________|    - робота се се движи право напред!
  }                                                       //    50sm      35sm      29sm      21sm      15sm      0sm
  switch (currentState)
  {
  case 1: // завой на 90 градуса на ляво
    Serial.println("ЛЯВ ЗАВОЙ");
    moveBackward();
    delay(100);
    speedLeft = LeftSpeed * 1.35;
    customTurnLeft();
    delay(750);
    moveBackward();
    delay(100);
    directionCompensation = false;
    break;
  case 2: // завой на 90 градуса надясно
    Serial.println("ДЕСЕН ЗАВОЙ");
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
    directionCompensation = false;
    break;
  case 3:  // леко въртене на дясно
    speedRight = RightSpeed * 2.55;
    customTurnRight();
    delay(50);
    break;
  case 4:  // леко въртене на ляво
    speedLeft = LeftSpeed * 2.55;
    customTurnLeft();
    delay(50);
    break;
  case 5:  // силно въртене на Ляво
    speedLeft = LeftSpeed * 2.55;
    customTurnLeft();
    delay(100);
    break;
  case 6:   // силно въртене на дясно
    speedRight = RightSpeed * 2.55;
    customTurnRight();
    delay(100);
    break;

  default:
    break;
  }
}
//==================================== VOID =====================================================

void moveForward()    //движение напред
{
  analogWrite(LEFT_FOR, abs(speedLeft));
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, abs(speedRight));
  analogWrite(RIGHT_BACK, LOW);
}

void moveBackward()   //движение назад
{
  analogWrite(LEFT_FOR, LOW);
  analogWrite(LEFT_BACK, abs(speedLeft));
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, abs(speedRight));
}

void customTurnLeft()  //въртене наляво
{
  analogWrite(LEFT_FOR, LOW);
  analogWrite(LEFT_BACK, speedLeft);
  analogWrite(RIGHT_FOR, speedLeft);
  analogWrite(RIGHT_BACK, LOW);
}

void customTurnRight()  //ръртене надясно
{
  analogWrite(LEFT_FOR, speedRight);
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, speedRight);
}

void stopMoving()   // спиране на движението
{
  analogWrite(LEFT_FOR, HIGH);
  analogWrite(LEFT_BACK, HIGH);
  analogWrite(RIGHT_FOR, HIGH);
  analogWrite(RIGHT_BACK, HIGH);
}

float getDistance(int servoAngle, int delayAfterServoMovement) //35,20,35,20,40
{
  float distance;
  myservo.write(servoAngle);
  //---------------------------150 millis
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
  //-----------------------
  pinMode(UltrasonicPin, OUTPUT);
  digitalWrite(UltrasonicPin, LOW);
  delayMicroseconds(2);
  digitalWrite(UltrasonicPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(UltrasonicPin, LOW);
  pinMode(UltrasonicPin, INPUT);
  distance = pulseIn(UltrasonicPin, HIGH) / 58.00;
  return distance;
}

// loop:
// move_servo
// slow_down
// read_sensor