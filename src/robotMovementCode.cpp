#include <HardwareSerial.h>
#include <Arduino.h>
#include <UcTTDcMotor.h>

#include "DualMotorControl.h"

// Define pins
const PwmPin MOTOR_PIN_LF = D3;
const PwmPin MOTOR_PIN_LR = D9;
const PwmPin MOTOR_PIN_RF = D10;
const PwmPin MOTOR_PIN_RR = D11;

const int TRIGGER_PIN = 7;
const int ECHO_PIN = 6;
const int ENCODER_PIN_L = 2;
const int ENCODER_PIN_R = 4;

// Ratio between pulse return time and distance in cm
const float DISTANCE_RATIO = 29.1;   
const int RPM_POLL_DURATION = 50; // How long to poll the encoders for RPM
const int RPM_POLL_FREQUENCY = 50;

// Define motor connections
UcTTDcMotor motorL(MOTOR_PIN_LF, MOTOR_PIN_LR); 
UcTTDcMotor motorR(MOTOR_PIN_RF, MOTOR_PIN_RR); 
DualMotorControl motorController(&motorL, &motorR, ENCODER_PIN_L, ENCODER_PIN_R); 
long duration;

//Code to get distance without a libary code based on  forum post https://forum.arduino.cc/t/code-to-use-ultrasonic-sensors-without-pulsein-function-or-newping-library/354239
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
    //Stops and cancles all movement if sensors pick up an object within 5cm
    int distance = pollDistance();
    
    if(distance <= 10){
        motorController.stop();
        Serial.print(distance);
        return;
    }

    //Moves Arduino based on 
    if (Serial.available() > 0) {
        char command = Serial.read();

        if (command == 'W' || command == 'w') {
            motorController.forward(50);
        } else if (command == 'S' || command == 's') {
            motorController.reverse(50);
        } else if (command == 'A' || command == 'a') {
            motorController.turnRobotByDegrees(1);
        } else if (command == 'D' || command == 'd') {
            motorController.turnRobotByDegrees(-1);
        } 
    } else {
        motorController.stop();
    }

    delay(50);
}