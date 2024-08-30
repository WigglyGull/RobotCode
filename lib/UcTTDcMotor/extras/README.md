# UcTTDcMotor

A simple Arduino Uno library that uses PWM acting on a [ZXBM5210 motor driver](https://www.diodes.com/assets/Datasheets/products_inactive_data/ZXBM5210.pdf) to operate a hobby-class ["TT" geared DC motor](https://www.adafruit.com/product/3777).

## Setup
The pins that are available for PWM control of motors are digital pins 3, 9, 10, and 11. These are represented
by the symbols `D3`, `D9`, `D10`, and `D11`. A single motor requires two of these pins to be assigned to it - one 
to produce"forward" rotation and the other to produce "reverse" rotation.  Thus a total of 2 motors can be controlled.

## Controlling the speed
The motor speed is a percentage from 0% (not moving) up to 100% (fastest possible rotation under the current load). 
It appears that a minimum speed of around 60% is required to start the motors moving, although lower speeds may 
work once the motor is actually moving.

The motors can be run in "forward" or "reverse" directions. It is recommended to stop the motor for at least half a 
second before starting the motor in the opposite direction. `UcTTDcMotor` automatically inserts a 500 millisecond stop
when it is commanded to change motor rotation direction.


## Example usage

```cpp
   UcTTDcMotor motor(D3, D9); // Instantiate a motor driver with forward and reverse pins
   void setup() {
     motor.init();  // Starts PWM @ ~31kHz
   }
   
   void loop() {
     motor.forward(60);  // 60% duty cycle on pin 3, 0% on pin 9
     delay(100);
     motor.reverse(60);  // 60% duty cycle on pin 9, 0% on pin 3 (with a 0.5s stop between forward and reverse)
   }
```

## Notes

### Blocking behaviour

The PWM library forces a 0.5 second interval between changes in the motor direction. This is a safety measure.
The pause is implemented using an Arduino `delay()` call, which is blocking - in other words if you change the motor direction
then your entire program will pause for 0.5s. Using a `delay()` inside a library like that is probably not good practice.
But it's a simple way to ensure that naive use of the PWM library will prevent damage to the motors.

If the 0.5 second pause is interfering with other things you're doing in your program, you have several options:

1. Call `motor.stop()` before changing motor direction. That will enable you to choose your own amount of time before
   starting the motor in the opposite direction to whatever it was previously doing.

2. Modify the library to better integrate with whatever timing mechanism your software is using (replace the `delay()` with
   something else).

3. Directly use the lower-level `UcMotorPwm` library to manage PWM, and handle all timing yourself.


### Pin and Timer constraints

The library **does not support PWM on pins 5 and 6** because those pins are associated with Timer 0,
which is also used for the `millis()` and `delay()` functions.

The [ZXBM5210 motor driver](https://www.diodes.com/assets/Datasheets/products_inactive_data/ZXBM5210.pdf) requires an input
PWM frequency of greater than 20kHz. The default Arduino PWM frequencies prodcued by `analogWrite()` are,
depending on the pin, 500Hz or 1kHz. The current implementation of `UcTTDcMotor` and the underlying `UcMotorPwm`
driver modify the timers that drive the hardware PWM generators to produce a PWM frequency of approximately 31kHz.
Modifying the behaviour of Timer 0 could break timing functionality.  See [](https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm)
for more information on PWM pins and timers.


## Known issues

The library does not currently support the ZXBM5210's "braking" mode.
