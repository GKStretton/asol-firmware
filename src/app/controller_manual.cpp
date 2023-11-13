#include "controller.h"
#include "../drivers/fs-i6.h"
#include "../middleware/sleep.h"
#include "../middleware/logger.h"
#include "../config.h"
#include "../calibration.h"
#include "../common/ik_algorithm.h"
#include "state_report.h"

// ikModeUpdate does ik logic on an x and y in range (-1,1).
void ikModeUpdate(State *s)
{
	if (FS_I6::GetSwitch(FS_I6::S2) == 2)
	{
		float dx = FS_I6::GetStick(FS_I6::RH);
		float dy = FS_I6::GetStick(FS_I6::RV);

		float sf = 0.05;
		s->target_x += sf * dx;
		s->target_y += sf * dy;
	}

	float ring, yaw;
	int code = getRingAndYawFromXY(s->target_x, s->target_y,
					 s->ringStepper.PositionToUnit(s->ringStepper.currentPosition()),
					 &ring, &yaw,
					 s->ringStepper.GetMinUnit(), s->ringStepper.GetMaxUnit());
	
	if (code != 0) {
		Logger::Error("error code fromgetRingAndYawFromXY, aborting");
		return;
	}

	if (ring < s->ringStepper.GetMinUnit() || ring > s->ringStepper.GetMaxUnit())
	{
		Logger::Warn("Unexpected ring value " + String(ring) + " detected, aborting ik!");
		return;
	}
	if (yaw < -20 || yaw > 20)
	{
		Logger::Warn("Potentially dangerous yaw value " + String(yaw) + " detected, aborting ik!");
		return;
	}

	Logger::Debug("Current ring = " + String(s->ringStepper.PositionToUnit(s->ringStepper.currentPosition())) + ", current yaw = " + String(s->yawStepper.PositionToUnit(s->yawStepper.currentPosition())));
	Logger::Debug("Target ring = " + String(ring) + ", final yaw = " + String(yaw));
	if (ENABLE_IK_ACTUATION)
	{
		s->ringStepper.moveTo(s->ringStepper.UnitToPosition(ring));
		s->yawStepper.moveTo(s->yawStepper.UnitToPosition(yaw));
	}
}

void Controller::manualUpdate(State *s, bool calibrating)
{
	StateReport_SetMode(machine_Mode_MANUAL);
	// Get inputs

	int sw1 = FS_I6::GetSwitch(FS_I6::S1);
	int sw2 = FS_I6::GetSwitch(FS_I6::S2);

	// Sleep

	if (sw1) {
		Sleep::Wake();
	}

	digitalWrite(STEPPER_SLEEP, sw1 ? HIGH : LOW);

	// Nothing to do if steppers asleep
	if (!sw1)
		return;

	// Main control

	float speedMult = 1600.0 * (calibrating ? 0.25 : 1.0); // if calibrating, go slow

	if (sw2 == 0 || sw2 == 1) {
		// manual + pipette

		float left_h = FS_I6::GetStick(FS_I6::LH);
		if (sw2 == 0) {
			s->ringStepper.setSpeed(speedMult * left_h);
		} else if (sw2 == 1) {
			s->pipetteStepper.setSpeed(-speedMult * left_h);
		}
		float right_h = FS_I6::GetStick(FS_I6::RH);
		s->yawStepper.setSpeed(speedMult * right_h);
		float right_v = FS_I6::GetStick(FS_I6::RV);
		s->pitchStepper.setSpeed(speedMult * right_v);
		s->zStepper.SetMinUnit(0);
	} else if (sw2 == 2) {
		// ik
		// block mode if pitch is low
		if (s->pitchStepper.PositionToUnit(s->pitchStepper.currentPosition()) >= CENTRE_PITCH - 1 &&
			s->zStepper.PositionToUnit(s->zStepper.currentPosition()) >= MIN_BOWL_Z)
		{
			ikModeUpdate(s);

			float left_h = FS_I6::GetStick(FS_I6::LH);
			s->pipetteStepper.setSpeed(-speedMult * left_h);

			s->pitchStepper.moveTo(s->pitchStepper.UnitToPosition(CENTRE_PITCH));
			s->zStepper.SetMinUnit(MIN_BOWL_Z);
		}
	}

	float left_v = FS_I6::GetStick(FS_I6::LV);
	s->zStepper.setSpeed(speedMult * left_v);
}