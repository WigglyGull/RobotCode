#include <Arduino.h>
#include <UcTTDcMotor.h>
#include <pins_arduino.h>

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

// Encoder configuration
const int RPM_POLL_DURATION = 2000; // How long to poll the encoders for RPM
const int RPM_POLL_FREQUENCY = 50;
const int INTERRUPTOR_SLOTS = 20; // Number of slots on the interruptor disc

// Define motor connections
UcTTDcMotor motorL(MOTOR_PIN_LF, MOTOR_PIN_LR); 
UcTTDcMotor motorR(MOTOR_PIN_RF, MOTOR_PIN_RR); 
long duration;
volatile int pulses;

void pulseISR() {
  pulses++;
}

double pollRpm(int encoderPin) {
  // Poll the L and R motors for current RPM
  // Blocking for a specified RPM_POLL_DURATION.
  pulses = 0;
  uint32_t end_time = millis() + RPM_POLL_DURATION;
  
  attachInterrupt(digitalPinToInterrupt(encoderPin), pulseISR, CHANGE);
  while (millis() < end_time);
  detachInterrupt(digitalPinToInterrupt(encoderPin));
  return (double) pulses / (1. * INTERRUPTOR_SLOTS);
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
  pinMode(ENCODER_PIN_L, INPUT_PULLUP);
  pinMode(ENCODER_PIN_R, INPUT_PULLUP);

  //Sets up motor
  motorL.init();  // Starts PWM @ ~31kHz
  motorR.init();  // Starts PWM @ ~31kHz
}
 
void loop()
{
  moveForward(60);
  Serial.print("L Wheel: ");
  Serial.println(pollRpm(ENCODER_PIN_L));
  //Serial.print("R Wheel: ");
  //Serial.println(pollRpm(ENCODER_PIN_R));
}
