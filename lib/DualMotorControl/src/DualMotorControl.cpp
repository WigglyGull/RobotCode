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

   // Stop the motors before beginning new motion
    stop();

    // Define encoder counting functions
    uint8_t flashesLeft = 0;
    uint8_t flashesRight = 0;
    bool lockLeft = 0;
    bool lockRight = 0;

    // Set the motors moving in the directions defined by reverse and pivot booleans, and turn on the associated LEDs
    if ( reverse ) {
        if ( pivot ) {
            digitalWrite(DualMotorControl::turnLed, 1);
            DualMotorControl::rightMotor->reverse(DualMotorControl::MOTOR_DUTY_CYCLE);
        } else {
            digitalWrite(DualMotorControl::forwardLed, 1);
            DualMotorControl::rightMotor->forward(DualMotorControl::MOTOR_DUTY_CYCLE);
        }
        DualMotorControl::leftMotor->reverse(DualMotorControl::MOTOR_DUTY_CYCLE);
        
    } else {
        if ( pivot ) {
            digitalWrite(DualMotorControl::turnLed, 1);
            DualMotorControl::rightMotor->forward(DualMotorControl::MOTOR_DUTY_CYCLE);
        } else {
            digitalWrite(DualMotorControl::forwardLed, 1);
            DualMotorControl::rightMotor->reverse(DualMotorControl::MOTOR_DUTY_CYCLE);
        }
        DualMotorControl::leftMotor->forward(DualMotorControl::MOTOR_DUTY_CYCLE);
        
    }

    // Move both motors, stopping each one after they have moved past the specified distance
    while (flashesLeft < steps || flashesRight < steps) { 
        // Increase L counter if a new flash has occurred
        if ( digitalRead(DualMotorControl::leftEncoder) ) {
            if (!lockLeft) {
                flashesLeft++;
                lockLeft = 1;
            }
        } else {
            lockLeft = 0;
        }

        // Increase R counter if a new flash has occurred
        if ( digitalRead(DualMotorControl::rightEncoder) ) {
            if (!lockRight) {
                flashesRight++;
                lockRight = 1;
            }
        } else {
            lockRight = 0;
        }

        // Stop the motors if enough flashes have occurred
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
   digitalWrite(DualMotorControl::turnLed, 1);
    if (degrees > 0) {
        stepDualMotor(0, 1, degrees * DualMotorControl::CLICKS_PER_DEGREE);
    } else {
        stepDualMotor(1, 1, -1 * degrees * DualMotorControl::CLICKS_PER_DEGREE);
    }
}


void DualMotorControl::forward(uint8_t speed)
{
    /* 
     * Move the robot forwards, ignoring encoders.
    */ 
    stop();
    digitalWrite(DualMotorControl::forwardLed, 1);
    DualMotorControl::leftMotor->forward((int) (speed * DualMotorControl::leftDutyOffset));
    DualMotorControl::rightMotor->reverse((int) (speed * DualMotorControl::rightDutyOffset));
}


void DualMotorControl::reverse(uint8_t speed)
{
    /* 
     * Move the robot backwards, ignoring encoders.
    */
    stop();
    digitalWrite(DualMotorControl::forwardLed, 1);
    DualMotorControl::leftMotor->reverse((int) (speed * DualMotorControl::leftDutyOffset));
    DualMotorControl::rightMotor->forward((int) (speed * DualMotorControl::rightDutyOffset));
}


void DualMotorControl::stop() 
{
    /* 
     * Stop both motors.
    */
    digitalWrite(DualMotorControl::forwardLed, 0);
    digitalWrite(DualMotorControl::turnLed, 0);
    DualMotorControl::leftMotor->stop();
    DualMotorControl::rightMotor->stop();
}

void DualMotorControl::setLeftDutyOffset(float offset)
{
    /*
    * Set a multiplier (between 0 and 1) for the duty cycle of the L motor
    * Allows compensation for 
    */
    DualMotorControl::leftDutyOffset = offset;
}

void DualMotorControl::setRightDutyOffset(float offset)
{
    /*
    * Set a multiplier (between 0 and 1) for the duty cycle of the L motor
    * Allows compensation for a drunk robot
    */
    DualMotorControl::rightDutyOffset = offset;
}

void DualMotorControl::turnRight(uint8_t speed)
{
    /*
    * Turns the robot to the right at a set speed
    */
    stop();
    digitalWrite(DualMotorControl::turnLed, 1);
    DualMotorControl::leftMotor->forward((int) (speed * DualMotorControl::leftDutyOffset));
    DualMotorControl::rightMotor->forward((int) (speed * DualMotorControl::rightDutyOffset));

}


void DualMotorControl::turnLeft(uint8_t speed)
{
    /*
    * Turns the robot to the left at a set speed
    */
    stop();
    digitalWrite(DualMotorControl::turnLed, 1);
    DualMotorControl::leftMotor->reverse((int) (speed * DualMotorControl::leftDutyOffset));
    DualMotorControl::rightMotor->reverse((int) (speed * DualMotorControl::rightDutyOffset));
}