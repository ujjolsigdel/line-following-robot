
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <NewPing.h>


//ir sensor
#define irRight 0
#define irCenter 1
#define irLeft 2

//ultrasonic sensor
#define TRIGGER_PIN 3
#define ECHO_PIN 4
#define max_distance 50

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *rightMotor = AFMS.getMotor(3);
// You can also make another motor on port M2
Adafruit_DCMotor *leftMotor = AFMS.getMotor(4);


Servo servo;

NewPing sonar(TRIGGER_PIN, ECHO_PIN, max_distance);

float duration, distance; 

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  pinMode(irLeft, INPUT);
  pinMode(irCenter, INPUT);
  pinMode(irRight, INPUT);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  rightMotor->setSpeed(100);
  rightMotor->run(FORWARD);
  // turn on motor
  rightMotor->run(RELEASE);

  leftMotor->setSpeed(100);
  leftMotor->run(FORWARD);
  // turn on motor
  leftMotor->run(RELEASE);
}

void loop() {
  //uint8_t i;


  digitalWrite(TRIGGER_PIN, LOW);  
	delayMicroseconds(2);  
	digitalWrite(TRIGGER_PIN, HIGH);  
	delayMicroseconds(10);  
	digitalWrite(TRIGGER_PIN, LOW);

  Serial.println(digitalRead(ECHO_PIN));

  //Serial.println(digitalRead(irLeft));

  boolean valRight = digitalRead(irRight);
  boolean valLeft = digitalRead(irLeft);
  
  if(valRight == HIGH && valLeft == HIGH){
    //objectAvoid();
    pitstop();
    rightMotor->run(FORWARD);
    leftMotor->run(BACKWARD);
    //delay(500);
  }
  else if(valRight == LOW && valLeft == HIGH){
    //objectAvoid();
    pitstop();
    rightMotor ->run(RELEASE);
    leftMotor ->run(BACKWARD);
  }
  else if(valRight == HIGH && valLeft == LOW){
    //objectAvoid();
    pitstop();
    rightMotor ->run(FORWARD);
    leftMotor ->run(RELEASE);
  }
  else if(valRight == LOW && valLeft == LOW){
    rightMotor ->run(RELEASE);
    leftMotor ->run(RELEASE);
  }
}

void pitstop(){
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = duration * 0.034 / 2;
  if(distance <= 10){
    rightMotor ->run(RELEASE);
    leftMotor ->run(RELEASE);
    delay(10000);
    rightMotor->run(FORWARD);
    leftMotor->run(BACKWARD);
  }
}

/**
void objectAvoid() {
  distance = getDistance();
  if (distance <= 15) {
    //stop
    Stop();
    Serial.println("Stop");

    lookLeft();
    lookRight();
    delay(100);
    if (rightDistance <= leftDistance) {
      //left
      object = true;
      turn();
      Serial.println("moveLeft");
    } else {
      //right
      object = false;
      turn();
      Serial.println("moveRight");
    }
    delay(100);
  }
  else {
    //forword
    Serial.println("moveforword");
    moveForward();
  }
}

int getDistance() {
  delay(50);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 100;
  }
  return cm;
}

int lookLeft () {
  //lock left
  servo.write(150);
  delay(500);
  leftDistance = getDistance();
  delay(100);
  servo.write(90);
  Serial.print("Left:");
  Serial.print(leftDistance);
  return leftDistance;
  delay(100);
}

int lookRight() {
  //lock right
  servo.write(30);
  delay(500);
  rightDistance = getDistance();
  delay(100);
  servo.write(90);
  Serial.print("   ");
  Serial.print("Right:");
  Serial.println(rightDistance);
  return rightDistance;
  delay(100);
}

void Stop() {
  rightMotor ->run(RELEASE);
  leftMotor ->run(RELEASE);
}

void moveForward() {
  rightMotor->run(FORWARD);
  leftMotor->run(BACKWARD);
}

void moveBackward() {
 rightMotor->run(BACKWARD);
 leftMotor->run(FORWARD);
}

void turn() {
  if (object == false) {
    Serial.println("turn Right");
    moveLeft();
    delay(700);
    moveForward();
    delay(800);
    moveRight();
    delay(900);
    if (digitalRead(irRight) == 1) {
      loop();
    } else {
      moveForward();
    }
  }
  else {
    Serial.println("turn left");
    moveRight();
    delay(700);
    moveForward();
    delay(800);
    moveLeft();
    delay(900);
    if (digitalRead(irLeft) == 1) {
      loop();
    } else {
      moveForward();
    }
  }
}
void moveRight() {
  rightMotor->run(RELEASE);
  leftMotor->run(BACKWARD);
}
void moveLeft() {
  rightMotor ->run(FORWARD);
  leftMotor ->run(RELEASE);
}
**/
