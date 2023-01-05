#include "UnitStepper.h"
#include <Arduino.h>
#include "../calibration.h"

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

void UnitStepper::Update() {
	updateLimitSwitch();
	enforceStepperLimits();

	if (positionWasSetLast_) {
		AccelStepper::run();
	} else {
		AccelStepper::runSpeed();
	}
}

void UnitStepper::updateLimitSwitch() {
	if (limitSwitchPin_ == 0) {
		// No switch
		return;
	}
	if (digitalRead(limitSwitchPin_)) {
		float oldSpeed = speed();
		setCurrentPosition(UnitToPosition(GetMinUnit()-CALIBRATION_GAP_SIZE));
		if (oldSpeed > 0)
		{
			setSpeed(oldSpeed);
		}
		MarkAsCalibrated();
	}
}

void UnitStepper::enforceStepperLimits() {
	if (!IsCalibrated()) {
		// Limits are meaningless without known position
		return;
	}

	// Block speeds if over max position
	if (PositionToUnit(currentPosition()) >= GetMaxUnit())
	{
		if (speed() > 0)
		{
			setSpeed(0);
		}
	}
	// Block speeds if under min position
	if (PositionToUnit(currentPosition()) <= GetMinUnit())
	{
		if (speed() < 0)
		{
			setSpeed(0);
		}
	}
}

void UnitStepper::SetLimitSwitchPin(uint8_t pin) {
	limitSwitchPin_ = pin;
}

void UnitStepper::moveTo(long p) {
	positionWasSetLast_ = true;
	AccelStepper::moveTo(p);
}

void UnitStepper::setSpeed(float s) {
	positionWasSetLast_ = false;
	// speed <> position bug workaround
	AccelStepper::stop();
	AccelStepper::setSpeed(s);
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

void UnitStepper::MarkAsNotCalibrated() {
	limitSwitchContacted_ = false;
}

void UnitStepper::MarkAsCalibrated() {
	limitSwitchContacted_ = true;
}

bool UnitStepper::IsCalibrated() {
	return limitSwitchContacted_ && !digitalRead(limitSwitchPin_);
}

void UnitStepper::MoveTarget(float d) {
	moveTo(targetPosition() + d);
}

bool UnitStepper::AtTarget() {
	// absolute value
	long d = this->distanceToGo();
	if (d < 0) d *= -1;
	return d <= UnitToPosition(atTargetUnitThreshold_);
}

bool UnitStepper::unitInRange(float a) {
	return a >= GetMinUnit() && a <= GetMaxUnit();
}

void UnitStepper::SetAtTargetUnitThreshold(float t) {
	atTargetUnitThreshold_ = t;
}

bool UnitStepper::HasLimitSwitchBeenPressed() {
	return limitSwitchContacted_;
}