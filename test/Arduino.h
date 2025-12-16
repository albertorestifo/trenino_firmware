// Mock Arduino.h for native testing
#pragma once

#include <stdint.h>

// Pin modes
#define INPUT 0
#define OUTPUT 1
#define INPUT_PULLUP 2

// Digital values
#define LOW 0
#define HIGH 1

// Analog pin definitions
#define A0 14
#define A1 15
#define A2 16
#define A3 17
#define A4 18
#define A5 19
#define A6 20
#define A7 21

// Mock Arduino functions (declarations only - implementations in test files)
void pinMode(uint8_t pin, uint8_t mode);
int analogRead(uint8_t pin);
int digitalRead(uint8_t pin);
void digitalWrite(uint8_t pin, uint8_t val);
void delayMicroseconds(unsigned int us);
unsigned long millis();
