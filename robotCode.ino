#include <HCSR04.h>
#include <UcTTDcMotor.h>

UcTTDcMotor motorL(D3, D9); // Instantiate a motor driver with forward and reverse pins
UcTTDcMotor motorR(D10, D11); // Instantiate a motor driver with forward and reverse pins

int trigPin = 2;    // Trigger
int echoPin = 3;    // Echo
long duration, cm, inches;
 
void setup() {
  //Serial Port begin
  Serial.begin (9600);
  //Define inputs and outputs
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}
 
void loop() {
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
  duration = pulseIn(echoPin, HIGH);
 
  // Convert the time into a distance
  cm = (duration/2) / 29.1;     // Divide by 29.1 or multiply by 0.0343
  inches = (duration/2) / 74;   // Divide by 74 or multiply by 0.0135
  
  Serial.print(inches);
  Serial.print("in, ");
  Serial.print(cm);
  Serial.print("cm");
  Serial.println();
  
  delay(250);
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
