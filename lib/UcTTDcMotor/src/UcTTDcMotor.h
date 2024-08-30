/*
 * Uses PWM acting on a ZXBM5210 motor driver to operate a "TT" DC motor.
 * 
 * Does not support PWM on pins 5 and 6 because those pins are
 * associated with Timer 0, which is also used for the millis()
 * and delay() functions. Modifying the behaviour of Timer 0
 * could break timing functionality. 
 * See https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm
 * 
  * Example usage:
 *   UcTTDcMotor motor(D3, D9); // Configures forward and reverse pins
 *   void setup() {
 *     motor.init();  // Starts PWM @ ~31kHz
 *   }
 *   
 *   void loop() {
 *     motor.forward(60);  // 60% duty cycle on pin 3, 0% on pin 9
 *     delay(100);
 *     motor.reverse(60);  // 60% duty cycle on pin 9, 0% on pin 3 (with a 0.5s stop between forward and reverse)
 *   }
 * 
 * 
 * Copyright 2023 Allan McInnes
 * Free software under a MIT-0 license (see LICENSE.txt or https://github.com/aws/mit-0)
 */
#ifndef UC_TT_DC_MOTOR
#define UC_TT_DC_MOTOR

#include "Arduino.h"
#include "UcMotorPwm.h"

class UcTTDcMotor {
  public:
    UcTTDcMotor(PwmPin forwardPin, PwmPin reversePin) : forwardPin(forwardPin), reversePin(reversePin) { };
    void init();
    void forward(uint8_t speed);
    void reverse(uint8_t speed);
    void stop();

  private:
    static bool pwmStarted;
    const int DIRECTION_CHANGE_TIME_MILLISECONDS = 500;
    PwmPin forwardPin;
    PwmPin reversePin;

    void setSpeedOnActivePin(uint8_t speed, PwmPin activePin, PwmPin inactivePin); 
};

#endif // UC_TT_DC_MOTOR
