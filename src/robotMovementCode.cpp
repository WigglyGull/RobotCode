#include <HardwareSerial.h>
#include <Arduino.h>
#include <UcTTDcMotor.h>

#include "DualMotorControl.h"

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
DualMotorControl motorController(&motorL, &motorR, ENCODER_PIN_L, ENCODER_PIN_R); 
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
    if (Serial.available() > 0) {
        char command = Serial.read();

        if (command == 'W' || command == 'w') {
            motorController.forward(50);
        } else if (command == 'S' || command == 's') {
            motorController.reverse(50);
        } else if (command == 'A' || command == 'a') {
            motorController.turnRobotByDegrees(-1);
        } else if (command == 'D' || command == 'd') {
            motorController.turnRobotByDegrees(1);
        } 
    } else {
        motorController.stop();
    }

    delay(50);
}