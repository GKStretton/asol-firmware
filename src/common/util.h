#pragma once
#include <Arduino.h>

void InitPin(uint8_t pin, byte v);
void Bound(int *number, int minimum, int maximum);

void SetDualRelay(uint8_t pin, bool on);
void SetSingleRelay(uint8_t pin, bool on);