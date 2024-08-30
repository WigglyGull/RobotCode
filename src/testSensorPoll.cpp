#include <Arduino.h>
#include <UcTTDcMotor.h>

// Define pins
const PwmPin MOTOR_PIN_LF = D3;
const PwmPin MOTOR_PIN_LR = D9;
const PwmPin MOTOR_PIN_RF = D10;
const PwmPin MOTOR_PIN_RR = D11;

const int TRIGGER_PIN = 6;
const int ECHO_PIN = 7;
const int ENCODER_PIN_L = 2;
const int ENCODER_PIN_R = 4;

// Ratio between pulse return time and distance in cm
const float DISTANCE_RATIO = 29.1;   
const int RPM_POLL_DURATION = 50; // How long to poll the encoders for RPM
const int RPM_POLL_FREQUENCY = 50;

// Define motor connections
UcTTDcMotor motorL(MOTOR_PIN_LF, MOTOR_PIN_LR); 
UcTTDcMotor motorR(MOTOR_PIN_RF, MOTOR_PIN_RR); 
long duration;


//returns distance from object in front in centermeters
int pollDistance()
{
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


int pollRpm() {
  // Poll the L and R motors for current RPM
  // This function is BLOCKING for a specified RPM_POLL_DURATION !!
  // Would need realtime support to avoid this. 

}

void stop(){
  motorL.stop();
  motorR.stop();
}


void moveForward(int speed)
{
  stop();
  motorL.forward(speed);
  motorR.reverse(speed); //is on backwards so is reversed
}

void moveBack(int speed){
  stop();
  motorL.reverse(speed);
  motorR.forward(speed);  //is on backwards so is reversed
}

void setup()
{
  //Serial Port begin
  Serial.begin (9600);

  //Define inputs and outputs
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  //Sets up motor
  motorL.init();  // Starts PWM @ ~31kHz
  motorR.init();  // Starts PWM @ ~31kHz
}
 
void loop()
{
  int distance = pollDistance();

  if(distance <= 10){
    Serial.println("Stoping");
    moveBack(60);
  } else {
    moveForward(60);
  }
}
