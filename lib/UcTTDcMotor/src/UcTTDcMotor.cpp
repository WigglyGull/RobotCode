/*
 * Copyright 2023 Allan McInnes
 * Free software under a MIT-0 license (see LICENSE.txt or https://github.com/aws/mit-0)
 */
#include "UcTTDcMotor.h"

bool UcTTDcMotor::pwmStarted = false;

void UcTTDcMotor::init() {
  if (!pwmStarted) {
      MotorPwm.begin();
      pwmStarted = true;
  }
  stop();
}

void UcTTDcMotor::forward(uint8_t speed) {
  setSpeedOnActivePin(speed, forwardPin, reversePin);
}

void UcTTDcMotor::reverse(uint8_t speed) {
  setSpeedOnActivePin(speed, reversePin, forwardPin);
}

void UcTTDcMotor::setSpeedOnActivePin(uint8_t speed, PwmPin activePin, PwmPin inactivePin) {
  if (MotorPwm.getDutyCycle(inactivePin) > 0) {
    stop();
    delay(DIRECTION_CHANGE_TIME_MILLISECONDS);
  }
  MotorPwm.setDutyCycle(activePin, speed);
}

void UcTTDcMotor::stop() {
  MotorPwm.setDutyCycle(forwardPin, 0);
  MotorPwm.setDutyCycle(reversePin, 0);
}
