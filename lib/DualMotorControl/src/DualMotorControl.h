/*
* Extension of Allan McInnes' UcTTDcMotor library.
*
* Adds the following functionality:
*   - "Fast" forward, reverse and turn functions for use with two motors.
*   - "Accurate" forward, reverse and turn functions for use with two motors and two rotary encoders.
*
* Copyright 2024 C. Varney, A. Walker, S. Baynes
* Free software under a MIT-0 license (see LICENSE.txt or https://github.com/aws/mit-0)
*/

#ifndef DUAL_MOTOR_CONTROLS_H

#include "Arduino.h"
#include "UcTTDcMotor.h"

class DualMotorControl {
  public:
  // pointers? which get dereferenced later? to prevent excessive memory use? i'm a GENIUS. a GOD even.
    DualMotorControl( UcTTDcMotor* leftMotor, UcTTDcMotor* rightMotor, int leftEncoder, int rightEncoder) : 
                      leftMotor(leftMotor), rightMotor(rightMotor), leftEncoder(leftEncoder), rightEncoder(rightEncoder) { };
    void forward(uint8_t speed);
    void reverse(uint8_t speed);
    void turnLeft(uint8_t speed);
    void turnRight(uint8_t speed);
    void stop();
    
    void moveRobot(int8_t steps);
    void turnRobotByDegrees(int8_t degrees);
    void setLeftDutyOffset(float offset);
    void setRightDutyOffset(float offset);


  private:
    UcTTDcMotor* leftMotor;
    UcTTDcMotor* rightMotor;
    int leftEncoder;
    int rightEncoder;
    float leftDutyOffset = 1;
    float rightDutyOffset = 1;

    const int MOTOR_DUTY_CYCLE = 60; // motors LOVE to stall below 50
    const float CLICKS_PER_DEGREE = 0.35;

    void stepMotor(UcTTDcMotor motor, int encoder, bool reverse, uint8_t steps);
    void stepDualMotor(bool reverse, bool pivot, uint8_t steps);
};

#endif // DUAL_MOTOR_CONTROL_H