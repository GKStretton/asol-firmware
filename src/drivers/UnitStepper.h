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

	// Call the relevant run function after checking limits
	void Update();
	// hides parent func so we can track whether position or speed setter was last called
	void moveTo(long position);
	void setSpeed(float s);

	bool unitInRange(float a);

	float GetMinUnit();
	float GetMaxUnit();

	void SetMinUnit(float val);
	void SetMaxUnit(float val);

	float PositionToUnit(float position);
	float UnitToPosition(float unit);

	void MarkAsCalibrated();
	void MarkAsNotCalibrated();
	// Returns true if limit switch has been pressed and released for this motor
	bool IsCalibrated();
	void SetLimitSwitchPin(uint8_t pin);
	// Returns true if limit switch has been pressed for this motor
	bool HasLimitSwitchBeenPressed();

	// (relative) Move the target position by an amount
	void MoveTarget(float d);

	// checks whether the stepper is at its target
	bool AtTarget();
	// Sets the threshold in units that position must be within of target for AtTarget to be true
	void SetAtTargetUnitThreshold(float t);

private:
	void updateLimitSwitch();
	void enforceStepperLimits();

	float microStepFactor_;
	float unitsPerFullStep_;
	float minUnit_;
	float maxUnit_;
	bool limitSwitchContacted_;
	// true if we last called moveTo, false if setSpeed was last called
	bool positionWasSetLast_;
	uint8_t limitSwitchPin_;
	float atTargetUnitThreshold_;
};