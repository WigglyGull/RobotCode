#include <UcTTDcMotor.h>

// Define pins
#define MOTOR_PIN_LF 3
#define MOTOR_PIN_LR 9
#define MOTOR_PIN_RF 10
#define MOTOR_PIN_RR 11

#define TRIGGER_PIN 6
#define ECHO_PIN 7
#define ENCODER_PIN_L 8
#define ENCODER_PIN_R 9

// Ratio between pulse return time and distance in cm
const float DISTANCE_RATIO = 29.1;   

// Define motor connections
UcTTDcMotor motorL(MOTOR_PIN_LF, MOTOR_PIN_LR); 
UcTTDcMotor motorR(MOTOR_PIN_RF, MOTOR_PIN_RR); 
long duration;

 
void setup() {
  //Serial Port begin
  Serial.begin (9600);

  //Define inputs and outputs
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  //Sets up motor
  motorL.init();  // Starts PWM @ ~31kHz
  motorR.init();  // Starts PWM @ ~31kHz
}
 
void loop() {
  int distance = pollDistance();

  if(distance <= 10){
    Serial.println("Stoping");
    moveBack(60);
  } else {
    moveForward(60);
  }
}

//returns distance from object in front in centermeters
int pollDistance(){
  // The sensor is triggered by a HIGH pulse of 10 or more microseconds.
  // Give a short LOW pulse beforehand to ensure a clean HIGH pulse:
  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(5);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);
 
  // Read the signal from the sensor: a HIGH pulse whose
  // duration is the time (in microseconds) from the sending
  // of the ping to the reception of its echo off of an object.
  pinMode(ECHO_PIN, INPUT);
  duration = pulseIn(ECHO_PIN, HIGH);
 
  // Convert the time into a distance
  return (duration/2) / DISTANCE_RATIO;
}

void moveForward(int speed){
  stop();
  motorL.forward(speed);
  motorR.reverse(speed); //is on backwards so is reversed
}

void moveBack(int speed){
  stop();
  motorL.reverse(speed);
  motorR.forward(speed);  //is on backwards so is reversed
}

void stop(){
  motorL.stop();
  motorR.stop();
}
