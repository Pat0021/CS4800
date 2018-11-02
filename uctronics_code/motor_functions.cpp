#ifndef MOTOR_FUNCTIONS
#define MOTOR_FUNCTIONS 

union {
  byte byteVal[4];
  float floatVal;
  long longVal;
} val;

union {
  byte byteVal[8];
  double doubleVal;
} valDouble;

union {
  byte byteVal[2];
  short shortVal;
} valShort;

unsigned char readBuffer(int index) {
  return isBluetooth ? bufferBt[index] : buffer[index];
}
void writeBuffer(int index, unsigned char c) {
  if (isBluetooth) {
    bufferBt[index] = c;
  } else {
    buffer[index] = c;
  }
}
void writeHead() {
  writeSerial(0xff);
  writeSerial(0x55);
}
void writeEnd() {
  Serial.println();
#if defined(__AVR_ATmega32U4__)
  Serial1.println();
#endif
}
void writeSerial(unsigned char c) {
  Serial.write(c);
#if defined(__AVR_ATmega32U4__)
  Serial1.write(c);
#endif
}
void readSerial() {
  isAvailable = false;
  if (Serial.available() > 0) {
    isAvailable = true;
    isBluetooth = false;
    serialRead = Serial.read();
  }
  if (isDetecte) {
    S = readPing();
    if (S <= TURN_DIST ) {
      leftMotor1.run(5); rightMotor1.run(5);//5-> stop
      leftMotor2.run(5); rightMotor2.run(5);//5-> stop
      leftMotor1.setSpeed(200); rightMotor1.setSpeed(200);
    }
  }
  while (myIR.available())
  {
    irValue =  myIR.read();
  }
  if (irValue == 0xFF46B9)  //forward
  {

    irValue = 0; isSmartMode = false;
    isDetecte = true;

    leftMotor1.run(1); rightMotor1.run(1);//1-> forward
    leftMotor2.run(1); rightMotor2.run(1);
    leftMotor1.setSpeed(200); rightMotor1.setSpeed(200);
    leftMotor2.setSpeed(200); rightMotor2.setSpeed(200);
  } else if (irValue == 0xFF15EA) { //backward

    irValue = 0; isSmartMode = false;
    isDetecte = false;

    leftMotor1.run(2); rightMotor1.run(2);//2-> backward
    leftMotor2.run(2); rightMotor2.run(2);
    leftMotor1.setSpeed(200); rightMotor1.setSpeed(200);
    leftMotor2.setSpeed(200); rightMotor2.setSpeed(200);
  } else if (irValue == 0xFF44BB) { // left

    irValue = 0; isSmartMode = false;
    isDetecte = false;

    leftMotor1.run(3); rightMotor1.run(3);//3-> left
    leftMotor2.run(3); rightMotor2.run(3);
    leftMotor1.setSpeed(200); rightMotor1.setSpeed(200);
    leftMotor2.setSpeed(200); rightMotor2.setSpeed(200);
  } else if (irValue == 0xFF43BC) { //right

    irValue = 0; isSmartMode = false;
    isDetecte = false;

    leftMotor1.run(4); rightMotor1.run(4);//4-> right
    leftMotor2.run(4); rightMotor2.run(4);
    leftMotor1.setSpeed(200); rightMotor1.setSpeed(200);
    leftMotor2.setSpeed(200); rightMotor2.setSpeed(200);
  } else if (irValue == 0xFF40BF) {  //stop

    irValue = 0; isSmartMode = false;
    isDetecte = false;

    leftMotor1.run(5); rightMotor1.run(5);//5-> stop
    leftMotor2.run(5); rightMotor2.run(5);
    leftMotor1.setSpeed(200); rightMotor1.setSpeed(200);
    leftMotor2.setSpeed(200); rightMotor2.setSpeed(200);
  }
}
/*
  ff 55 len idx action device port  slot  data a
  0  1  2   3   4      5      6     7     8
*/
void parseData() {
  isStart = false;
  int idx = readBuffer(3);
  command_index = (uint8_t)idx;
  int action = readBuffer(4);
  int device = readBuffer(5);
  switch (action) {
    case GET: {
        writeHead();
        writeSerial(idx);
        readSensor(device);
        writeEnd();
      }
      break;
    case RUN: {
        runModule(device);
        callOK();
      }
      break;
    case RESET: {
        //reset
        leftMotor1.run(5); rightMotor1.run(5);
        leftMotor2.run(5); rightMotor2.run(5);
        leftMotor1.setSpeed(0); rightMotor1.setSpeed(0);
        leftMotor2.setSpeed(0); rightMotor2.setSpeed(0);
        neckControllerServoMotor.write(90);
        delay(100);
        neckControllerServoMotor.detach();
        delay(100);
        isSmartMode = false;
        myIR.begin();
        callOK();
      }
      break;
    case START: {
        //start
        callOK();
      }
      break;
  }
}
void callOK() {
  writeSerial(0xff);
  writeSerial(0x55);
  writeEnd();
}
void sendByte(char c) {
  writeSerial(1);
  writeSerial(c);
}
void sendString(String s) {
  int l = s.length();
  writeSerial(4);
  writeSerial(l);
  for (int i = 0; i < l; i++) {
    writeSerial(s.charAt(i));
  }
}
void sendFloat(float value) {
  writeSerial(0x2);
  val.floatVal = value;
  writeSerial(val.byteVal[0]);
  writeSerial(val.byteVal[1]);
  writeSerial(val.byteVal[2]);
  writeSerial(val.byteVal[3]);
}
void sendShort(double value) {
  writeSerial(3);
  valShort.shortVal = value;
  writeSerial(valShort.byteVal[0]);
  writeSerial(valShort.byteVal[1]);
}
void sendDouble(double value) {
  writeSerial(2);
  valDouble.doubleVal = value;
  writeSerial(valDouble.byteVal[0]);
  writeSerial(valDouble.byteVal[1]);
  writeSerial(valDouble.byteVal[2]);
  writeSerial(valDouble.byteVal[3]);
}
short readShort(int idx) {
  valShort.byteVal[0] = readBuffer(idx);
  valShort.byteVal[1] = readBuffer(idx + 1);
  return valShort.shortVal;
}
float readFloat(int idx) {
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx + 1);
  val.byteVal[2] = readBuffer(idx + 2);
  val.byteVal[3] = readBuffer(idx + 3);
  return val.floatVal;
}
long readLong(int idx) {
  val.byteVal[0] = readBuffer(idx);
  val.byteVal[1] = readBuffer(idx + 1);
  val.byteVal[2] = readBuffer(idx + 2);
  val.byteVal[3] = readBuffer(idx + 3);
  return val.longVal;
}
int readPing()
{
  // establish variables for duration of the ping,
  // and the distance result in inches and centimeters:
  long duration, cm;
  // The PING))) is triggered by a HIGH pulse of 2 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  pinMode(TRIG_PIN, OUTPUT);
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(5);
  digitalWrite(TRIG_PIN, LOW);
  pinMode(ECHO_PIN, INPUT);
  duration = pulseIn(ECHO_PIN, HIGH);
  // convert the time into a distance
  cm = duration / 29 / 2;;
  return cm ;
}

void turn() {
  leftMotor1.run(5); rightMotor1.run(5);//5-> stop
  leftMotor2.run(5); rightMotor2.run(5);
  leftMotor1.setSpeed(0); rightMotor1.setSpeed(0);
  leftMotor2.setSpeed(0); rightMotor2.setSpeed(0);
  neckControllerServoMotor.write(150);
  delay(500);
  S = readPing();
  Sleft = S;
  neckControllerServoMotor.write(90);
  delay(500);
  neckControllerServoMotor.write(30);
  delay(500);
  S = readPing();
  Sright = S;
  neckControllerServoMotor.write(90);
  delay(500);
  if (Sleft <= TURN_DIST && Sright <= TURN_DIST) {
    leftMotor1.run(2); rightMotor1.run(2);//2-> backward
    leftMotor2.run(2); rightMotor2.run(2);
    leftMotor1.setSpeed(200); rightMotor1.setSpeed(200);
    leftMotor2.setSpeed(200); rightMotor2.setSpeed(200);
    delay(500);
    int x = random(1);
    if (x = 0) {
      leftMotor1.run(4); rightMotor1.run(4);//4-> right
      leftMotor2.run(4); rightMotor2.run(4);//4-> right
      leftMotor1.setSpeed(200); rightMotor1.setSpeed(200);
      leftMotor2.setSpeed(200); rightMotor2.setSpeed(200);
    }
    else {
      leftMotor1.run(3); rightMotor1.run(3);//3-> left
      leftMotor2.run(3); rightMotor2.run(3);
      leftMotor1.setSpeed(200); rightMotor1.setSpeed(200);
      leftMotor2.setSpeed(200); rightMotor2.setSpeed(200);
    }
    delay(500);
  } else {
    if (Sleft >= Sright) {
      leftMotor1.run(3); rightMotor1.run(3);//3-> left
      leftMotor2.run(3); rightMotor2.run(3);
      leftMotor1.setSpeed(200); rightMotor1.setSpeed(200);
      leftMotor2.setSpeed(200); rightMotor2.setSpeed(200);
    } else {
      leftMotor1.run(4); rightMotor1.run(4);//4-> right
      leftMotor2.run(4); rightMotor2.run(4);//4-> right
      leftMotor1.setSpeed(200); rightMotor1.setSpeed(200);
      leftMotor2.setSpeed(200); rightMotor2.setSpeed(200);
    }
    delay(500);
  }
}
void runModule(int device) {
  //0xff 0x55 0x6 0x0 0x1 0xa 0x9 0x0 0x0 0xa
  int port = readBuffer(6);
  int pin = port;
  switch (device) {
    case SERVO: {
        int angle = readBuffer(6);
        if (angle >= 0 && angle <= 180)
        {
          neckControllerServoMotor.attach(SERVO_PIN);
          neckControllerServoMotor.write(angle);
        }
      }
      break;
    case ROBOTCAR: {
        int directionMode = readBuffer(6);
        int motorSpeed = readBuffer(8);
        leftMotor1.run(directionMode);   rightMotor1.run(directionMode);
        leftMotor2.run(directionMode);   rightMotor2.run(directionMode);
        leftMotor1.setSpeed(motorSpeed); rightMotor1.setSpeed(motorSpeed);
        leftMotor2.setSpeed(motorSpeed); rightMotor2.setSpeed(motorSpeed);
      }
      break;
  }
}

void readSensor(int device) {
  /**************************************************
      ff 55 len idx action device port slot data a
       0  1  2   3   4      5      6    7    8
  ***************************************************/
  float value = 0.0;
  int port, slot, pin;
  port = readBuffer(6);
  pin = port;
  switch (device) {
    case ULTRASONIC_SENSOR: {
        value = readPing();
        sendFloat(value);
      }
      break;
  }
}
void moveTrack(void)
{
  int temp = 0, num1 = 0, num2 = 0, num3 = 0;
  while (1) {
    if ((Serial.available() > 0))
    {
      isTrackMode = false;
      break;
    }
    num1 = digitalRead(leftSensor);
    num2 = digitalRead(middleSensor);
    num3 = digitalRead(rightSensor);
    if ((num2 == 0) && (num1 == 0) && (num3 == 0)) {
      leftMotor1.run(5); rightMotor1.run(5);//5-> stop
      leftMotor2.run(5); rightMotor2.run(5);
      leftMotor1.setSpeed(0); rightMotor1.setSpeed(0);
      leftMotor2.setSpeed(0); rightMotor2.setSpeed(0);
      continue;
    } else if ( (num1 == 0) && num3 == 1) { //go to right
      leftMotor1.run(3); rightMotor1.run(3);//3-> left
      leftMotor2.run(3); rightMotor2.run(3);
      leftMotor1.setSpeed(0); rightMotor1.setSpeed(180);
      leftMotor2.setSpeed(0); rightMotor2.setSpeed(180);
      while (1) {
        if ((Serial.available() > 0))
        {
          isTrackMode = false;
          break;
        }
        num2 = digitalRead(middleSensor);
        if (num2) {
          leftMotor1.run(3); rightMotor1.run(3);//3-> left
          leftMotor2.run(3); rightMotor2.run(3);
          leftMotor1.setSpeed(0); rightMotor1.setSpeed(180);
          leftMotor2.setSpeed(0); rightMotor2.setSpeed(180);
          continue;
        }
        else
          break;
      }
    } else if ((num3 == 0) && (num1 == 1)) { // go to left
      leftMotor1.run(4); rightMotor1.run(4);//4-> right
      leftMotor2.run(4); rightMotor2.run(4);
      leftMotor1.setSpeed(180); rightMotor1.setSpeed(0);
      leftMotor2.setSpeed(180); rightMotor2.setSpeed(0);
      while (1) {
        if ((Serial.available() > 0))
        {
          isTrackMode = false;
          break;
        }
        num2 = digitalRead(middleSensor);
        if (num2) {
          leftMotor1.run(4); rightMotor1.run(4);//4-> right
          leftMotor2.run(4); rightMotor2.run(4);
          leftMotor1.setSpeed(180); rightMotor1.setSpeed(0);
          leftMotor2.setSpeed(180); rightMotor2.setSpeed(0);
          continue;
        }
        else
          break;
      }
    }
    else
    {
      leftMotor1.run(1); rightMotor1.run(1);//1-> forward
      leftMotor2.run(1); rightMotor2.run(1);
      leftMotor1.setSpeed(180); rightMotor1.setSpeed(180);
      leftMotor2.setSpeed(180); rightMotor2.setSpeed(180);
    }
  }
}

void servo_right(void)
{
  neckControllerServoMotor.attach(10); delay(10);
  int servotemp = neckControllerServoMotor.read();
  servotemp -= servoStep;
  servo_horizontal(servotemp);
  delay(50);
  neckControllerServoMotor.detach();
  delay(50);
  myIR.begin();

}
void servo_left(void)
{
  neckControllerServoMotor.attach(10); delay(10);
  int servotemp = neckControllerServoMotor.read();
  servotemp += servoStep;
  servo_horizontal(servotemp);
  delay(50);
  neckControllerServoMotor.detach();
  delay(50);
  myIR.begin();
}

void servo_horizontal(int corner)
{
  int i = 0;
  byte cornerX = neckControllerServoMotor.read();
  if (cornerX > corner) {
    for (i = cornerX; i > corner; i = i - servoStep) {
      \
      neckControllerServoMotor.write(i);
      servoXPoint = i;
      delay(50);
    }
  }
  else {
    for (i = cornerX; i < corner; i = i + servoStep) {
      neckControllerServoMotor.write(i);
      servoXPoint = i;
      delay(50);
    }
  }
  neckControllerServoMotor.write(corner);
  servoXPoint = corner;
}

#endif
