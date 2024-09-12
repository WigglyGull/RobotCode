# DualMotorControl

Extension of Allan McInnes' UcTTDcMotor library, adding further abstraction for controlling dual motors and support for more accurate motion with rotary encoders.

## Adds the following functionality:
 - "Fast" forward, reverse and turn functions for use with two motors.
 - "Accurate" forward, reverse and turn functions for use with two motors and two rotary encoders.
 - The ability to set offsets the power of each motor, used by all motion functions, to offset unbalanced velocities on each wheel.  
 - Indicator LED support