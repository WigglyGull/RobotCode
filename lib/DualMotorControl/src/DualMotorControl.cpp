/*
* Extension of Allan McInnes' UcTTDcMotor library, adding further abstraction for controlling dual
* motors and support for more accurate motion with rotary encoders.
*
* Adds the following functionality:
*   - "Fast" forward, reverse and turn functions for use with two motors.
*   - "Accurate" forward, reverse and turn functions for use with two motors and two rotary encoders.
*
* Copyright 2024 C. Varney, A. Walker, S. Baynes
* Free software under a MIT-0 license (see LICENSE.txt or https://github.com/aws/mit-0)
*/

#include "Arduino.h"
#include "UcTTDcMotor.h"
#include "DualMotorControl.h"


void DualMotorControl::stepMotor(UcTTDcMotor motor, int encoder, bool reverse, uint8_t steps)
{
    /* 
     * Move one motor a set number of steps.
     * Blocking function!
    */ 
   stop();

    uint8_t flashes = 0;
    if ( reverse ) {
        motor.reverse(DualMotorControl::MOTOR_DUTY_CYCLE);
    } else {
        motor.forward(DualMotorControl::MOTOR_DUTY_CYCLE);
    }

    while (flashes < steps) { // can't be good
    if ( digitalRead(encoder) ) {
        flashes++;
        while ( digitalRead(encoder) ); // really can't be good
    }
  }
  
  motor.stop();
}


void DualMotorControl::stepDualMotor(bool reverse, bool pivot, uint8_t steps)
{
    /* 
     * Move both motors a set number of steps.
     * Blocking function!
    */ 
    stop();

    uint8_t flashesLeft = 0;
    uint8_t flashesRight = 0;
    bool lockLeft = 0;
    bool lockRight = 0;

    if ( reverse ) {
        if ( pivot ) {
            DualMotorControl::leftMotor->forward(DualMotorControl::MOTOR_DUTY_CYCLE);
        } else {
            DualMotorControl::leftMotor->reverse(DualMotorControl::MOTOR_DUTY_CYCLE);
        }
        DualMotorControl::rightMotor->forward(DualMotorControl::MOTOR_DUTY_CYCLE);
    } else {
        if ( pivot ) {
            DualMotorControl::rightMotor->forward(DualMotorControl::MOTOR_DUTY_CYCLE);
        } else {
            DualMotorControl::rightMotor->reverse(DualMotorControl::MOTOR_DUTY_CYCLE);
        }
        DualMotorControl::leftMotor->forward(DualMotorControl::MOTOR_DUTY_CYCLE);
    }

    while (flashesLeft < steps || flashesRight < steps) { 
        if ( digitalRead(DualMotorControl::leftEncoder) ) {
            if (!lockLeft) {
                flashesLeft++;
                lockLeft = 1;
            }
        } else {
            lockLeft = 0;
        }

        if ( digitalRead(DualMotorControl::rightEncoder) ) {
            if (!lockRight) {
                flashesRight++;
                lockRight = 1;
            }
        } else {
            lockRight = 0;
        }

        if (flashesLeft >= steps) {
            DualMotorControl::leftMotor->stop();
        }
        if (flashesRight >= steps) {
            DualMotorControl::rightMotor->stop();
        }

  }
}


void DualMotorControl::moveRobot(int8_t steps)
{
    /* 
     * Move robot a set number of steps - an abstraction of stepDualMotor
    */ 
    if (steps > 0) {
        stepDualMotor(0, 0, steps);
    } else {
        stepDualMotor(1, 0, -1 * steps);
    }
}


void DualMotorControl::turnRobotByDegrees(int8_t degrees)
{
    /* 
     * Pivot robot a defined number of degrees.
    */ 
    if (degrees > 0) {
        Serial.print("Left");
        stepDualMotor(0, 1, degrees * DualMotorControl::CLICKS_PER_DEGREE);
    } else {
        Serial.print("Right");
        stepDualMotor(1, 1, degrees * DualMotorControl::CLICKS_PER_DEGREE);
    }
}

void DualMotorControl::forward(uint8_t speed)
{
    /* 
     * Move the robot forwards, ignoring encoders.
    */ 
    stop();
    DualMotorControl::leftMotor->forward(speed);
    DualMotorControl::rightMotor->reverse(speed);
}


void DualMotorControl::reverse(uint8_t speed)
{
    /* 
     * Move the robot backwards, ignoring encoders.
    */
    stop();
    DualMotorControl::leftMotor->reverse(speed);
    DualMotorControl::rightMotor->forward(speed);
}


void DualMotorControl::stop() 
{
    /* 
     * Stop both motors.
    */
    DualMotorControl::leftMotor->stop();
    DualMotorControl::rightMotor->stop();
}