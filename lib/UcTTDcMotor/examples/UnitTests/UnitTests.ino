#include <UcMotorPwm.h>
#include <UcTTDcMotor.h>

UcTTDcMotor motorA(D3, D11);

void setup() {
  Serial.begin(9600);
  runPwmUnitTests();
  runMotorUnitTests();
}

void loop() {

}


/* ------------------ Test functions -------------------------- */
// The following is a simple implementation of
// automated testing. For more complex tests, consider a testing
// framework like AUnit or ArduinoUnit.
void testAssertEqual(int actual, int expected) {
  if (expected == actual) {
    Serial.println("Ok");
  }
  else {
    Serial.print("Test failure: expected ");
    Serial.print(expected);
    Serial.print(" but got ");
    Serial.println(actual);
  }
}

void runPwmUnitTests() {
  Serial.println("---- Running Pwm Tests ----");
    // Configure PWM output pins to make sure PWM signal is not output
  pinMode(D3, INPUT);
  pinMode(D9, INPUT);
  pinMode(D10, INPUT);
  pinMode(D11, INPUT);
  
  // Start all pins with duty cycle of 0%
  MotorPwm.setDutyCycle(D3, 0);
  MotorPwm.setDutyCycle(D9, 0);
  MotorPwm.setDutyCycle(D10, 0);
  MotorPwm.setDutyCycle(D11, 0);
  
  testGetDutyCycles();        
  testTimer1GetSet();
  testTimer2GetSet();
  testInvalidPin();
}

void testGetDutyCycles() {
    testAssertEqual(MotorPwm.getDutyCycle(D3), 0);
    testAssertEqual(MotorPwm.getDutyCycle(D9), 0);
    testAssertEqual(MotorPwm.getDutyCycle(D10), 0);
    testAssertEqual(MotorPwm.getDutyCycle(D11), 0);
}

void testTimer1GetSet() {
    MotorPwm.setDutyCycle(D9, 65);
    testAssertEqual(MotorPwm.getDutyCycle(D9), 65);
    MotorPwm.setDutyCycle(D9, 100);
    testAssertEqual(MotorPwm.getDutyCycle(D9), 100);
    MotorPwm.setDutyCycle(D9, 128);
    testAssertEqual(MotorPwm.getDutyCycle(D9), 27);  // Should wrap around
    MotorPwm.setDutyCycle(D9, 0);
    testAssertEqual(MotorPwm.getDutyCycle(D9), 0);
}

void testTimer2GetSet() {
    MotorPwm.setDutyCycle(D3, 65);
    testAssertEqual(MotorPwm.getDutyCycle(D3), 65);
    MotorPwm.setDutyCycle(D3, 100);
    testAssertEqual(MotorPwm.getDutyCycle(D3), 100);
    MotorPwm.setDutyCycle(D3, 128);
    testAssertEqual(MotorPwm.getDutyCycle(D3), 27);  // Should wrap around    
    MotorPwm.setDutyCycle(D3, 0);
    testAssertEqual(MotorPwm.getDutyCycle(D3), 0);
}

void testInvalidPin() {
    MotorPwm.setDutyCycle(5, 100);
    testAssertEqual(MotorPwm.getDutyCycle(5), 0);
}

void runMotorUnitTests() {
  Serial.println("---- Running Motor Tests ----");
  motorA.init();
  
  motorA.forward(90);
  testAssertEqual(MotorPwm.getDutyCycle(D3), 90);
  testAssertEqual(MotorPwm.getDutyCycle(D11), 0);
  delay(1);
  motorA.forward(100);
  testAssertEqual(MotorPwm.getDutyCycle(D3), 100);
  testAssertEqual(MotorPwm.getDutyCycle(D11), 0);
  delay(1);
  motorA.reverse(70);
  testAssertEqual(MotorPwm.getDutyCycle(D3), 0);
  testAssertEqual(MotorPwm.getDutyCycle(D11), 70);  
  delay(1);
  motorA.reverse(30);
  testAssertEqual(MotorPwm.getDutyCycle(D3), 0);
  testAssertEqual(MotorPwm.getDutyCycle(D11), 30);   

  motorA.stop();
}
