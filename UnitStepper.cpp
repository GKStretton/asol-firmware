#include "UnitStepper.h"

UnitStepper::UnitStepper(
    uint8_t stepPin,
    uint8_t dirPin,
    float microStepFactor,
    float unitsPerFullStep,
    float minUnit,
    float maxUnit
)
    :
    AccelStepper(AccelStepper::DRIVER, stepPin, dirPin),
    microStepFactor_(microStepFactor),
    unitsPerFullStep_(unitsPerFullStep),
    minUnit_(minUnit),
    maxUnit_(maxUnit)
{
}

float UnitStepper::GetMinUnit() {
    return minUnit_;
}

float UnitStepper::GetMaxUnit() {
    return maxUnit_;
}

void UnitStepper::SetMinUnit(float val) {
    minUnit_ = val;
}

void UnitStepper::SetMaxUnit(float val) {
    maxUnit_ = val;
}

float UnitStepper::PositionToUnit(float position) {
    return position / microStepFactor_ * unitsPerFullStep_;
}

float UnitStepper::UnitToPosition(float unit) {
    return unit / unitsPerFullStep_ * microStepFactor_;
}

void UnitStepper::MarkAsCalibrated() {
    calibrated_ = true;
}

bool UnitStepper::IsCalibrated() {
    return calibrated_;
}

void UnitStepper::MoveTarget(float d) {
    moveTo(targetPosition() + d);
}