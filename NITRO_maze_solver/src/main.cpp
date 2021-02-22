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

// константите започват с главна буква, а всяка следваща дума с главна буква
const int LeftIrAvoidancePin = 12;
const int RightIrAvoidancePin = A5;
const int UltrasonicPin = 3;
const int RgbPin = 2;
// и тези пинове трябва да са константи - не се променят
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

// Тук започват ГЛОБАЛНИТЕ променливи, които се използват в кода:
// променливите започват с малка буква, а всяка следваща дума с главна буква
// ....
float maxDistance = 130.0;
int speedLeft = LeftSpeed;
int speedRight = RightSpeed;
bool directionCompensation = false;
// Тук инициализираме обектите:
Servo myservo;

// Тук слагаме прототипите на функциите:
void moveForward();
void turnRight();
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
  // states:
  int currentState = 0; //? Правим проверките универсални - за следване на стена отдясно или отляво в зависимост от избрания WallFollowingSide - параметър по-горе.
                        // Състоянието (state) след прочитане на разстоянията може да бъде:
                        // 1 Приближили сме стена отпред (<= FrontDistanceTreshold), отдясно/отляво има стена (< SideCorridorTreshold) - завой на 180 градуса
                        // 2 Пррближили сме стена отпред (<= FrontDistanceTreshold), има коридор надясно/наляво (>= SideCorridorTreshold) - завой на 90 градуса надясно/наляво
                        // 3 Напред стената е далече (> FrontDistanceTreshold), има коридор в дясно/ляво (>= SideCorridorTreshold) - завой на 90 градуса надясно/наляво
                        // 4 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), в рамките на +- CenterLineTolerance от централната линия сме - движение право напред
                        // 5 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), на повече от + SharpTurnTreshold от централната линия (по-близо до лявата/дясната стена) сме - движение напред с остър завой наляво/надясно
                        // 6 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), на повече от - SharpTurnTreshold от централната линия сме (по-близо до дясната/лявата стена) сме - движение напред с остър завой надясно/наляво
                        // 7 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), на повече от + tolerance от централната линия (по-близо до лявата/дясната стена) сме - движение напред със завой леко наляво/надясно
                        // 8 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), на повече от - tolerance от централната линия сме (по-близо до дясната/лявата стена) сме - движение напред със завой леко надясно/наляво

  sideDistance = getDistance(SideServoAngle, SideServoDelay);
  frontDistance = getDistance(FrontServoAngle, FrontServoDelay);

  Serial.print("frontDistance - ");
  Serial.println(frontDistance);
  Serial.print("sideDistance - ");
  Serial.println(sideDistance);

  if (frontDistance <= 25.0) //) //Стената отпред е близко SideCorridorTreshold
  {
    digitalWrite(LedPin, HIGH);
    currentState = 1; // turn
                      // TODO ??????????????? mejdu 25 i 50 ??????????????????????????????
    //------------------------------------------
    //if (sideDistance < SideCorridorTreshold)
    //{
    // 1 Приближили сме стена отпред (<= FrontDistanceTreshold), отдясно/отляво има стена (< SideCorridorTreshold)
    // - завой на 90 градуса
    //  currentState = 1; // turn 90 degrees
    // }
    // else if (sideDistance >= SideCorridorTreshold )
    // {
    // 2 Пррближили сме стена отпред (<= FrontDistanceTreshold), има коридор надясно/наляво (>= SideCorridorTreshold)
    // - завой на 90 градуса надясно/наляво
    //   currentState = 2; // turn 90 degrees
    // }
    //----------------------------------------------
  }
  if (frontDistance >= 25.0) //Стената отпред е далече
  {
    if (sideDistance >= 50.0) //SideCorridorTreshold
    {
      // 3 Напред стената е далече (> FrontDistanceTreshold), има коридор в дясно/ляво (>= SideCorridorTreshold)
      // - завой на 90 градуса надясно/наляво
      Serial.println("SET case =  3 ");
      currentState = 3; // turn 90 degrees
    }
    else if (sideDistance > 21.0 && sideDistance < 29.0) //(sideDistance < 23 && sideDistance > 27sideDistance < WallToCorridorMiddle + CenterLineTolerance && sideDistance > WallToCorridorMiddle - CenterLineTolerance)
    {
      // 4 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), в рамките на +- CenterLineTolerance
      // от централната линия сме  - движение право напред
      Serial.println("SET case = 4 ");
      currentState = 4; //Close to the centerline - go forwad
    }
    else if (sideDistance < 35.0 && sideDistance >= 29.0) //(sideDistance >= WallToCorridorMiddle + SharpTurnTreshold)
    {
      // 5 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), на повече от + SharpTurnTreshold от
      // централната линия (по-близо до лявата/дясната стена) сме - движение напред с остър завой наляво/надясно
      Serial.println("SET case = 5 ");
      currentState = 5; //Close to the other wall - hard turn to centerline
    }
    else if (sideDistance > 15.0 && sideDistance <= 21.0) //(sideDistance <= WallToCorridorMiddle - SharpTurnTreshold)
    {
      // 6 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), на повече от - SharpTurnTreshold
      //  от централната линия сме (по-близо до дясната/лявата стена) сме - движение напред с остър завой наляво/надясно
      Serial.println("SET case = 6 ");
      currentState = 6; //Close to the wall we are following - hard turn to centerline
    }
    else if (sideDistance <= 15.0)
    {
      // 8 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), на повече от - tolerance
      //  въртим се с while докаро изправим робота
      Serial.println("SET case = 7 ");
      currentState = 7;
    }
    else if (sideDistance >= 35.0 && sideDistance < 50.0)
    {
      // 8 Има стена отдясно/отляво (< SideCorridorTreshold), напред е свободно (> FrontDistanceTreshold), на повече от - tolerance
      //  въртим се с while докаро изправим робота
      Serial.println("SET case = 8 ");
      currentState = 8;
    }
    if (frontDistance < 30.0 && sideDistance < 15.0)
    {
      currentState = 9;
    }
  }
  switch (currentState)
  {
  case 1: /* завой на 90 градуса */
    Serial.println("case 1");
    stopMoving();
    speedLeft = LeftSpeed * 1.8;
    customTurnLeft();
    delay(550);
    directionCompensation = false;
    break;
  // case 2: /* завой на 90 градуса надясно/наляво */
  // Serial.println("case 2");
  //       stopMoving();
  //   moveBackward();
  //   delay(200);
  //   customTurnRight();
  //   delay(500);
  //   directionCompensation = false;
  //   break;
  case 3: /* завой на 90 градуса надясно/наляво */
    Serial.println("case 3");
    stopMoving();
    moveBackward();
    delay(200);
    for (size_t i = 0; i < 4; i++)
    {
      speedLeft = LeftSpeed;
      speedRight = RightSpeed;
      moveForward();
      delay(20);
      speedLeft = 255;
      speedRight = 0;
      moveForward();
      delay(170);
    }
    directionCompensation = false;
    break;
  case 4:
    Serial.println("case 4");
    stopMoving();
    delay(20);
    speedLeft = LeftSpeed;
    speedRight = RightSpeed;
    moveForward();
    directionCompensation = false;
    /* движение право напред */
    break;
  case 5:
    Serial.println("case 5");
    /* движение напред с остър завой наляво/надясно */
    speedLeft = LeftSpeed * 2.0;
    customTurnLeft();
    delay(20);    
    break;
  case 6:
    Serial.println("case 6");
    customTurnRight();
    speedRight = RightSpeed * 2.0;
    delay(20);
    /* движение напред с остър завой надясно/наляво */
    break;
  case 7:
    Serial.println("case 7");
    speedLeft = LeftSpeed * 2.55;
    customTurnLeft();
    delay(20);
    break;
  case 8:
    Serial.println("case 8");
    speedRight = RightSpeed * 2.55;
    customTurnRight();
    delay(20);
    break;
  case 9:
    Serial.println("case 9");
    speedLeft = LeftSpeed * 1.6;
    customTurnLeft();
    delay(400);
    break;
  default:
    break;
  }
}
//==================================== VOID =====================================================

void moveForward()
{
  analogWrite(LEFT_FOR, abs(speedLeft));
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, abs(speedRight));
  analogWrite(RIGHT_BACK, LOW);
}

void turnRight()
{
  analogWrite(LEFT_FOR, 180);
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, 0);
}

void moveBackward()
{
  analogWrite(LEFT_FOR, LOW);
  analogWrite(LEFT_BACK, abs(speedLeft));
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, abs(speedRight));
}

void customTurnLeft()
{
  analogWrite(LEFT_FOR, LOW);
  analogWrite(LEFT_BACK, speedLeft);
  analogWrite(RIGHT_FOR, speedLeft);
  analogWrite(RIGHT_BACK, LOW);
}

void customTurnRight()
{
  analogWrite(LEFT_FOR, speedRight);
  analogWrite(LEFT_BACK, LOW);
  analogWrite(RIGHT_FOR, LOW);
  analogWrite(RIGHT_BACK, speedRight);
}

void stopMoving()
{
  analogWrite(LEFT_FOR, HIGH);
  analogWrite(LEFT_BACK, HIGH);
  analogWrite(RIGHT_FOR, HIGH);
  analogWrite(RIGHT_BACK, HIGH);
}

float getDistance(int servoAngle, int delayAfterServoMovement)
{
  float distance;
  myservo.write(servoAngle);
  //delay(delayAfterServoMovement);
  //---------------- забавено движение 150 millis.
  speedLeft = LeftSpeed;
  speedRight = RightSpeed;
  moveForward();
  delay(35);
  stopMoving();
  delay(40);
  moveForward();
  delay(35);
  stopMoving();
  delay(40);
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