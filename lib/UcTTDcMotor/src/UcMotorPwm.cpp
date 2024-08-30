/*
 * Copyright 2023 Allan McInnes 
 * Free software under a MIT-0 license (see LICENSE.txt or https://github.com/aws/mit-0)
 */
#include "UcMotorPwm.h"

UcMotorPwm MotorPwm;  // Statically-allocated motor PWM driver

void UcMotorPwm::begin() {
    // See https://ww1.microchip.com/downloads/en/DeviceDoc/Atmel-7810-Automotive-Microcontrollers-ATmega328P_Datasheet.pdf   
    // for details of registers and settings
    
    // Timer 1 - pins D9 and D10
    TCCR1A = bit(WGM10); // Phase correct PWM with max value of 255 (8-bit mode)
    TCCR1A |= bit(COM1A1) | bit(COM1B1); // Non-inverting mode for both A and B pins
    TCCR1B = bit(CS10); // No prescaling ==> ~31kHz PWM in phase correct mode 

    // Timer 2  - pins D3 and D11
    TCCR2A = bit(WGM00); // Phase correct PWM with max value of 255 (8-bit mode)
    TCCR2A |= bit(COM2A1) | bit(COM2B1); // Non-inverting mode for both A and B pins
    TCCR2B = bit(CS20); // No prescaling ==> ~31kHz PWM in phase correct mode  

    // Start all pins with duty cycle of 0%
    setDutyCycle(D3, 0);
    setDutyCycle(D9, 0);
    setDutyCycle(D10, 0);
    setDutyCycle(D11, 0);

    // Configure PWM output pins to make PWM signal available
    pinMode(D3, OUTPUT);
    pinMode(D9, OUTPUT);
    pinMode(D10, OUTPUT);
    pinMode(D11, OUTPUT);    
}


void UcMotorPwm::setDutyCycle(PwmPin pin, uint8_t dutyCyclePercent) {
  volatile uint8_t* comparisonRegisterAddress = comparisonRegisterForPin(pin); 
  if (comparisonRegisterAddress != NULL) {
    *comparisonRegisterAddress = dutyCycleToComparisonRegisterValue(dutyCyclePercent);
  }
}


int UcMotorPwm::getDutyCycle(PwmPin pin) {
  uint8_t dutyCyclePercent = 0;
  volatile uint8_t* comparisonRegisterAddress = comparisonRegisterForPin(pin); 
  if (comparisonRegisterAddress != NULL) {
    dutyCyclePercent = comparisonRegisterValueToDutyCycle(*comparisonRegisterAddress);
  }
  return dutyCyclePercent;
}


uint8_t UcMotorPwm::dutyCycleToComparisonRegisterValue(uint8_t dutyCyclePercent) {
  return integerRationalScaling(dutyCyclePercent, MAX_COMPARE_VALUE, MAX_PERCENT);
}


uint8_t UcMotorPwm::comparisonRegisterValueToDutyCycle(uint8_t registerValue) {
  return integerRationalScaling(registerValue, MAX_PERCENT, MAX_COMPARE_VALUE); 
}


// Use integer arithmetic to scale an integer by a rational number
uint8_t UcMotorPwm::integerRationalScaling(uint8_t value, int scalingNumerator, int scalingDenominator) {
   // Avoid floating-point arithmetic by multiplying value by numerator before performing division. 
   // Add denominator/2 prior to division to make integer-arithmetic truncation behave like rounding
   // (equivalent to adding 0.5 to the quotient).
   int scaledValue = scalingNumerator * value;
   int rounding = scalingDenominator / 2;
   return (scaledValue + rounding) / scalingDenominator; 
}


volatile uint8_t* UcMotorPwm::comparisonRegisterForPin(PwmPin pin) {
  switch (pin) {
    case D3: return &OCR2B; break;
    case D9: return (uint8_t*)&OCR1A; break; // Must cast pointer to 16-bit register into 8-bit block
    case D10: return (uint8_t*)&OCR1B; break;
    case D11: return &OCR2A; break;
    default: return NULL;
  }
}
