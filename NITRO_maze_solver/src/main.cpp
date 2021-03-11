#include <NewPing.h>

#define TRIG A0      // ............
#define ECHO A1      // define front Ultrasonic
#define MAX_DIST 200 // ............

#define TRIG_L A4      // ............
#define ECHO_L A5      // define left Ultrasonic
#define MAX_DIST_L 200 // ............

#define TRIG_R A2      // ............
#define ECHO_R A3      // define right Ultrasonic
#define MAX_DIST_R 200 // ............

#define RIGHT_PWM 10 9  // define left motor PWM
#define RIGHT_FOR 8 5  // define left motor forwart
#define RIGHT_BACK 7 4 // define left motor backward
#define STBY 6       // define STBY pyn
#define LEFT_FOR 5   // define right motor forward
#define LEFT_BACK 4   // define right motor backward
#define LEFT_PWM 9  // define right motor PWM

const int LEFT_SPEED = 60;
const int RIGHT_SPEED = 60;

int leftSpeed = LEFT_SPEED;
int rightSpeed = RIGHT_SPEED;

NewPing sonar(TRIG, ECHO, MAX_DIST);
NewPing sonar_L(TRIG_L, ECHO_L, MAX_DIST_L);
NewPing sonar_R(TRIG_R, ECHO_R, MAX_DIST_R);

void moveForward();
void moveBackward();
void customTurnLeft();
void customTurnRight();
void stopMoving();

void setup()
{
    pinMode(STBY, OUTPUT);
    pinMode(RIGHT_FOR, OUTPUT);
    pinMode(RIGHT_BACK, OUTPUT);
    pinMode(RIGHT_PWM, OUTPUT);
    pinMode(LEFT_FOR, OUTPUT);
    pinMode(LEFT_BACK, OUTPUT);
    pinMode(LEFT_PWM, OUTPUT);

    digitalWrite(STBY, HIGH);
    //unsigned int frontDistance;// = sonar.ping_cm();
    // unsigned int leftDistance; //= sonar_L.ping_cm();
    //unsigned int rightDistance;// = sonar_R.ping_cm();
    Serial.begin(9600);
    digitalWrite(RIGHT_PWM, RIGHT_SPEED);
    digitalWrite(LEFT_PWM, LEFT_SPEED);
   // customTurnRight();
   // delay(5000);
}

/*-------------------------------------------------*/

void loop()

{
    unsigned int frontDistance;
    unsigned int leftDistance;
    unsigned int rightDistance;
    int currentState = 0;

    leftDistance = sonar_L.ping_cm(); //read left Ultrasonic sensor
    Serial.print("LEFT-");
    Serial.print(leftDistance);
    if (leftDistance == 0)
        leftDistance = 255;

    frontDistance = sonar.ping_cm(); //read front Ultrasonic sensor
    Serial.print("    FRONT-");
    Serial.print(frontDistance);
    if (frontDistance == 0)
        frontDistance = 255;        
        
    rightDistance = sonar_R.ping_cm(); //read right Ultrasonic sensor
    Serial.print("    RIGHT-");
    Serial.println(rightDistance);
    if (rightDistance == 0)
        rightDistance = 255; 
    
          

    delay(250);

    if (frontDistance <= 15.0) //the front wall is neal, turn left 90 degrees
    {
        currentState = 1;
    }

    if (frontDistance >= 15.0) //the front wall is far away
    {
        if (rightDistance >= 30.0) //the right wall is far away
        {
            currentState = 2; // turn right 90 degrees
        }
        else
        {
            currentState = 3;
        }
    }

    switch (currentState)
    {
    case 1: //завой на 90 градуса наляво
        Serial.println("ЛЯВ ЗАВОЙ");
        moveBackward();
        delay(100);
        leftSpeed = LEFT_SPEED * 1.35;
        rightSpeed = RIGHT_SPEED * 1.35;
        customTurnLeft();
        delay(750);
        moveBackward();
        delay(100);
        break;

    case 2: // завой на 90 градуса надясно
        Serial.println("ДЕСЕН ЗАВОЙ");
        leftSpeed = LEFT_SPEED * 2;
        rightSpeed = RIGHT_SPEED * .5;
        moveForward();
        delay(750);
        leftSpeed = LEFT_SPEED ;
        rightSpeed = RIGHT_SPEED ;
        moveForward();
        delay(200);
        break;

    case 3:
        Serial.println("Движение направо");
        rightSpeed = RIGHT_SPEED + leftDistance;
        leftSpeed = LEFT_SPEED * rightDistance;
        break;

    default:
        break;
    }

    moveForward(); 
    
}
void moveForward()
{
    digitalWrite(LEFT_FOR, HIGH);
    digitalWrite(LEFT_BACK, LOW);
    digitalWrite(RIGHT_FOR, HIGH);
    digitalWrite(RIGHT_BACK, LOW);
    analogWrite(RIGHT_PWM, rightSpeed);
    analogWrite(LEFT_PWM, leftSpeed);
}

void moveBackward()
{
    digitalWrite(LEFT_FOR, LOW);
    digitalWrite(LEFT_BACK, HIGH);
    digitalWrite(RIGHT_FOR, LOW);
    digitalWrite(RIGHT_BACK, HIGH);
    analogWrite(RIGHT_PWM, rightSpeed);
    analogWrite(LEFT_PWM, leftSpeed);
}

void customTurnRight()
{
    digitalWrite(LEFT_FOR, LOW);
    digitalWrite(LEFT_BACK, HIGH);
    digitalWrite(RIGHT_FOR, HIGH);
    digitalWrite(RIGHT_BACK, LOW);
    analogWrite(RIGHT_PWM, rightSpeed);
    analogWrite(LEFT_PWM, leftSpeed);
}

void customTurnLeft()
{
    digitalWrite(LEFT_FOR, HIGH);
    digitalWrite(LEFT_BACK, LOW);
    digitalWrite(RIGHT_FOR, LOW);
    digitalWrite(RIGHT_BACK, HIGH);
    analogWrite(RIGHT_PWM, rightSpeed);
    analogWrite(LEFT_PWM, leftSpeed);
}

void stopMoving()
{
    digitalWrite(LEFT_FOR, LOW);
    digitalWrite(LEFT_BACK, LOW);
    digitalWrite(RIGHT_FOR, LOW);
    digitalWrite(RIGHT_BACK, LOW);
}