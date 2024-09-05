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

const int FORWARD_LED = 13;
const int TURN_LED = 12;
const int HAZARD_LED = 8;
const int LEDS[3] = {FORWARD_LED, TURN_LED, HAZARD_LED};

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
  pinMode(FORWARD_LED, OUTPUT);
  pinMode(TURN_LED, OUTPUT);
  pinMode(HAZARD_LED, OUTPUT);

  //Sets up motor
  motorL.init();  // Starts PWM @ ~31kHz
  motorR.init();  // Starts PWM @ ~31kHz
}

bool hasStopped = false;
void loop() {
    //Turn leds off
    for (int i = 0; i < 2; i++){
        digitalWrite(LEDS[i], LOW);  
    }

    //Stops and cancles all movement if sensors pick up an object within 5cm
    int distance = pollDistance();
    if(distance <= 10){
        if(!hasStopped){
            hasStopped = true;
            Serial.print("Hazard detected please move");
        }
        motorController.stop();
        Serial.read(); //stops it reading input and excuting it after the hazard is removed
        digitalWrite(HAZARD_LED, HIGH); 
        return;
    }
    
    //Moves Arduino based on WASD
    if (Serial.available() > 0) {
        char command = Serial.read();
        int speed = isUpperCase(command) ? 40 : 80;
        hasStopped = false;

        if (command == 'W' || command == 'w') {
            motorController.forward(speed);
            digitalWrite(FORWARD_LED, HIGH); 
        } else if (command == 'S' || command == 's') {
            motorController.reverse(speed);
        } else if (command == 'A' || command == 'a') {
            motorController.turnRobotByDegrees(1);
            digitalWrite(TURN_LED, HIGH); 
        } else if (command == 'D' || command == 'd') {
            motorController.turnRobotByDegrees(-1);
            digitalWrite(TURN_LED, HIGH);
        } 
    } else {
        motorController.stop();
    }

    delay(50);
}