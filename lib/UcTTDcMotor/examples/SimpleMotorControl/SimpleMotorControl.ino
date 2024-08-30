#include <UcTTDcMotor.h>

// Create separate motor controllers and assign corresponding pins
UcTTDcMotor motorA(D3, D11);
UcTTDcMotor motorB(D9, D10);

void setup() {
  // Start motor PWM
  motorA.init();
  motorB.init();
}

void loop() {
  // Run each motor forward and backwards and various speeds
  motorA.forward(90);
  motorB.reverse(65);
  delay(1000);
  motorA.forward(100);
  motorB.reverse(100);
  delay(1000);
  motorA.reverse(70);
  motorB.forward(70);
  delay(1000);
  motorA.reverse(30);
  motorB.forward(30);
  delay(1000);
  motorA.stop();
  motorB.stop();
  delay(1000);
}
