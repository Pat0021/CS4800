/*

    Software Engineering Project - CS 4800
    California Polytechnical University: Pomona
    Charlson So, Lynn Nguyen, Vinh Duc Le, Carlos Olea, Kyle Ah-Tye

*/

#include <UCMotor.h>
#include <Servo.h>
#include "UCNEC.h"
#include "motor_functions.cpp"
typedef struct MeModule
{
  int device;
  int port;
  int slot;
  int pin;
  int index;
  float values[3];
} MeModule;

String mVersion = "0d.01.105";
boolean isAvailable = false;
boolean isBluetooth = false;
boolean isDetecte = false;
int len = 52;
char buffer[52];
char bufferBt[52];
byte index = 0;
byte dataLen;
boolean isStart = false;
char serialRead;
// define the device ID
#define ROBOTCAR 54
#define ULTRASONIC_SENSOR 55
#define SERVO 56

#define GET 1
#define RUN 2
#define RESET 4
#define START 5
unsigned char prevc = 0;
double lastTime = 0.0;
uint8_t keyPressed = 0;
uint8_t command_index = 0;

bool isSmartMode  = true;
bool isIrMode  = true;
bool isTrackMode = false;
unsigned int S;
unsigned int Sleft;
unsigned int Sright;

uint32_t irValue = 0;

#define TURN_DIST 40

#define TRIG_PIN A2
#define ECHO_PIN A3
#define SERVO_PIN 10

#define leftSensor    A0
#define middleSensor  A1
#define rightSensor   13

//The center Angle of two steering engines.
byte servoXCenterPoint = 88;
//The accuracy of servo
byte servoStep = 4;
//The current Angle of the two steering engines is used for retransmission
byte servoXPoint = 0;

UC_DCMotor leftMotor1(3, MOTOR34_64KHZ);
UC_DCMotor rightMotor1(4, MOTOR34_64KHZ);
UC_DCMotor leftMotor2(1, MOTOR34_64KHZ);
UC_DCMotor rightMotor2(2, MOTOR34_64KHZ);

Servo neckControllerServoMotor;
UCNEC myIR(2);
void setup() {
  pinMode(ECHO_PIN, INPUT); //Set the connection pin output mode Echo pin
  pinMode(TRIG_PIN, OUTPUT);//Set the connection pin output mode trog pin
  neckControllerServoMotor.attach(SERVO_PIN);
  neckControllerServoMotor.write(90);
  delay(2000);
  neckControllerServoMotor.detach();
  delay(100);
  myIR.begin();
  Serial.begin(115200);
  Serial.print("Version: ");
  Serial.println(mVersion);
}
void loop() {
  readSerial();
  if (isAvailable) {
    unsigned char c = serialRead & 0xff;
    if (c == 0x55 && isStart == false) {
      if (prevc == 0xff) {
        index = 1;
        isStart = true; isSmartMode = false;
      }
    } else {
      prevc = c;
      if (isStart) {
        if (index == 2) {
          dataLen = c;
        } else if (index > 2) {
          dataLen--;
        }
        writeBuffer(index, c);
      }
    }
    index++;
    if (index > 51) {
      index = 0;
      isStart = false;
    }
    if (isStart && dataLen == 0 && index > 3) {
      isStart = false;
      parseData();
      index = 0;
    } else if (!isStart) {
      if (serialRead >= 1 && serialRead <= 5) { //0x01->forward  0x02->backward  0x03->left  0x04-> right  0x05->stop
        if (serialRead == 1)  {
          isDetecte = true;
        }
        else  {
          isDetecte = false;
        }
        leftMotor1.run(serialRead); rightMotor1.run(serialRead);
        leftMotor2.run(serialRead); rightMotor2.run(serialRead);
        leftMotor1.setSpeed(200); rightMotor1.setSpeed(200);
        leftMotor2.setSpeed(200); rightMotor2.setSpeed(200);
        neckControllerServoMotor.detach();
        delay(100);
        isSmartMode = false;
        myIR.begin();
      }
      if (serialRead == 0x06) { //automatic obstacle avoidance
        neckControllerServoMotor.attach(SERVO_PIN);
        neckControllerServoMotor.write(90);
        isSmartMode = true;
      }
      if (serialRead == 0x07) { //tracking line mode enable
        neckControllerServoMotor.attach(SERVO_PIN);
        neckControllerServoMotor.write(90);
        delay(100);
        neckControllerServoMotor.detach();
        delay(100);
        isSmartMode = false;
        myIR.begin();
        isTrackMode = true; isDetecte = false;
      }
      if (serialRead == 0x08) {
        servo_left();
      }
      if (serialRead == 0x09) {
        servo_right();
      }
      if (serialRead == 0x11) {
        Serial.write(0xAA);
        Serial.print("{\"version\":\"K0072\"}");
        Serial.write(0xAB);
      }

      if (serialRead == 0x10) { //connect to bluetooth
        neckControllerServoMotor.attach(SERVO_PIN);
        neckControllerServoMotor.write(90);
        delay(100);
        neckControllerServoMotor.detach();
        delay(100);
        isSmartMode = false;
        myIR.begin();
        leftMotor1.run(0x05); rightMotor1.run(0x05);
        leftMotor2.run(0x05); rightMotor2.run(0x05);
      }
    }
  }
  if (isSmartMode) {
    S = readPing();
    if (S <= TURN_DIST ) {
      isIrMode = false;
      neckControllerServoMotor.attach(SERVO_PIN);
      neckControllerServoMotor.write(90);
      turn();
    } else if (S > TURN_DIST) {
      leftMotor1.run(1); rightMotor1.run(1);//1-> forward
      leftMotor2.run(1); rightMotor2.run(1);
      leftMotor1.setSpeed(200); rightMotor1.setSpeed(200);
      leftMotor2.setSpeed(200); rightMotor2.setSpeed(200);
      if (isIrMode == false) {
        isIrMode = true;
        neckControllerServoMotor.detach();
        delay(100);
        myIR.begin();
      }
    }
  }
  if (isTrackMode) {
    moveTrack();
  }
}
