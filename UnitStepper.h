#pragma once
#include <AccelStepper.h>

/*
To calculate unitsPerFullStep:
    1. Zero the stepper.
    2. Move some known units U.
    3. Observe position P in logs.
    4. Calculate full-step position = P/microStepFactor.
    5. Calculate unitsPerFull = U / (P / microStepFactor) 
*/
class UnitStepper: public AccelStepper {
public:
    UnitStepper(
        uint8_t stepPin,
        uint8_t dirPin,
        float microStepFactor,
        float unitsPerFullStep,
        float minUnit,
        float maxUnit
    );

    float GetMinUnit();
    float GetMaxUnit();

    float PositionToUnit(float position);
    float UnitToPosition(float unit);

private:
    float microStepFactor_;
    float unitsPerFullStep_;
    float minUnit_;
    float maxUnit_;
};