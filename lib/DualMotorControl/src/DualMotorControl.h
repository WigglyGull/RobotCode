/*
* Extension of Allan McInnes' UcTTDcMotor library, adding further abstraction for controlling dual
* motors and support for more accurate motion with rotary encoders.
*
* Adds the following functionality:
*   - "Fast" forward, reverse and turn functions for use with two motors.
*   - "Accurate" forward, reverse and turn functions for use with two motors and two rotary encoders.
*   - The ability to set offsets the power of each motor, used by all motion functions, to offset
*     unbalanced velocities on each wheel.  
*   - Indicator LED support
*
* Copyright 2024 C. Varney, A. Walker, S. Baynes
* Free software under a MIT-0 license (see LICENSE.txt or https://github.com/aws/mit-0)
*/

#ifndef DUAL_MOTOR_CONTROLS_H

#include "Arduino.h"
#include "UcTTDcMotor.h"

#define MOTOR

class DualMotorControl {
  public:
    // Initialiser
    DualMotorControl( UcTTDcMotor* leftMotor, UcTTDcMotor* rightMotor, int leftEncoder, int rightEncoder, int forwardLed, int turnLed) : 
                      leftMotor(leftMotor), rightMotor(rightMotor), leftEncoder(leftEncoder), rightEncoder(rightEncoder), forwardLed(forwardLed), turnLed(turnLed) { };
    
    // Basic movement functions 
    void forward(uint8_t speed);
    void reverse(uint8_t speed);
    void turnLeft(uint8_t speed);
    void turnRight(uint8_t speed);
    void stop();
    
    // Fine movement functions
    void moveRobot(int8_t steps);
    void turnRobotByDegrees(int8_t degrees);

    // Duty offset adjustment functions, for improving straight-line motion as required
    void setLeftDutyOffset(float offset);
    void setRightDutyOffset(float offset);


  private:
    // Constants
    const int MOTOR_DUTY_CYCLE = 40;      // Motor duty cycle to use for movement where the encoders are used
    const float CLICKS_PER_DEGREE = 0.23; // How many clicks the encoder moves per degree of ROBOT rotation in turn modes

    // Private variables to store parsed arguments
    UcTTDcMotor* leftMotor;
    UcTTDcMotor* rightMotor;
    int leftEncoder;
    int rightEncoder;
    float leftDutyOffset = 1;
    float rightDutyOffset = 1;
    int forwardLed;
    int turnLed;

    // Internal functions to control motion
    void stepMotor(UcTTDcMotor motor, int encoder, bool reverse, uint8_t steps);
    void stepDualMotor(bool reverse, bool pivot, uint8_t steps);
};

#endif // DUAL_MOTOR_CONTROL_H