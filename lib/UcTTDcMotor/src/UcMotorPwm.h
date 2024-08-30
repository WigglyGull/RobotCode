/*
 * The UC Motor PWM library is a minimal implementation of
 * hardware PWM at a frequency greater than the 
 * default for ATmega328-based Arduinos, but which is suitable for
 * PWM control of motors ( > 20kHz to reduce audible noise).
 * 
 * Does not support PWM on pins 5 and 6 because those pins are
 * associated with Timer 0, which is also used for the millis()
 * and delay() functions. Modifying the behaviour of Timer 0
 * could break timing functionality. 
 * See https://docs.arduino.cc/tutorials/generic/secrets-of-arduino-pwm
 * 
 * Example usage:
 *   #include "UcMotorPWm.h"
 *   
 *   void setup() {
 *     MotorPwm.begin();  // Starts PWM on pins 3, 9, 10, and 11
 *   }
 *   
 *   void loop() {
 *     MotorPwm.setDutyCycle(D3, 60);  // 60% duty cycle on pin 3
 *     delay(100);
 *     MotorPwm.setDutyCycle(D3, 80);  // 80% duty cycle on pin 3
 *   }  
 *  
 * Many thanks to Jonathan Edwards (ENEL ME student 2023) for performing 
 * initial PWM configuration experiments with the ZXBM5210 motor driver. 
 * The final PWM configuration differs somewhat from what Jonathan came up 
 * with, but his work made arriving at this configurations much faster.
 *  
 * Copyright 2023 Allan McInnes
 * Free software under a MIT-0 license (see LICENSE.txt or https://github.com/aws/mit-0)
 */
#ifndef UC_MOTOR_PWM
#define UC_MOTOR_PWM

#include "Arduino.h"

#if defined(ARDUINO_ARCH_AVR)

  // Pins that can run PWM
  enum PwmPin { D3 = 3, D9 = 9, D10 = 10, D11 = 11 };

  class UcMotorPwm {
    public:
      // Start PWM on all
      void begin();

      // Query the duty cycle on a pin. Returns a duty cycle % value.
      int getDutyCycle(PwmPin pin);

      // Modify the duty cycle on a pin
      void setDutyCycle(PwmPin pin, uint8_t dutyCyclePercent);

    private:
      const int MAX_PERCENT = 100;
      const int MAX_COMPARE_VALUE = 255;

      uint8_t dutyCycleToComparisonRegisterValue(uint8_t dutyCyclePercent);
      uint8_t comparisonRegisterValueToDutyCycle(uint8_t registerValue);
      uint8_t integerRationalScaling(uint8_t value, int scalingNumerator, int scalingDenominator);
      volatile uint8_t* comparisonRegisterForPin(PwmPin pin);
  };

  // Make the statically-allocated driver available for use by sketches
  extern UcMotorPwm MotorPwm;

#else
  #error "This library only supports boards with an AVR processor."
#endif

#endif // UC_MOTOR_PWM
