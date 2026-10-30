#pragma once
#include "protocol.h"
#include "stepper.h"
namespace StepperService {
typedef void (*Sender)(const uint8_t*, size_t);
void init(Sender sender);
bool canApplyModules();
uint8_t bootFaultAddress();
bool onPacket(const uint8_t* bytes, size_t length);
void update();
bool reset();
bool conflicts(const Protocol::Configure& config);
bool reservesPin(uint8_t pin);
}
