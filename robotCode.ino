#include <HCSR04.h>
#include <UcTTDcMotor.h>

UcTTDcMotor motorL(D3, D9); // Instantiate a motor driver with forward and reverse pins
UcTTDcMotor motorR(D10, D11); // Instantiate a motor driver with forward and reverse pins

int trigPin = 6;    // Trigger
int echoPin = 7;    // Echo
long duration, cm, inches;
 
void setup() {
  //Serial Port begin
  Serial.begin (9600);

  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  //Sets up motor
  motorL.init();  // Starts PWM @ ~31kHz
  motorR.init();  // Starts PWM @ ~31kHz
}
 
void loop() {
  int distance = detectDistance;
  moveForward(80);
  if(distance <= 5){
    motorL.stop();
    motorR.stop();
  }
}

//returns distance from object in front in centermeters
int detectDistance(){
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(trigPin, LOW);
  delayMicroseconds(5);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(echoPin, INPUT);
  duration = ulseIn(echoPin, HIGH);
 
  // Convert the time into a distance
  return (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
}

void moveForward(int speed){
  motorL.forward(speed);
  motorR.reverse(speed); //is on backwards so is reversed
  motorL.stop();
  motorR.stop();
}

void moveBack(int speed){
  motorL.reverse(speed);
  motorR.forward(speed);  //is on backwards so is reversed
  motorL.stop();
  motorR.stop();
}
