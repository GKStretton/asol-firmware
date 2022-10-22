#include <simpleRPC.h>
#include "switch.h"
#include "stage.h"
#include "fluid.h"
#include "config.h"
#include "logger.h"
#include "sleep.h"
#include "util.h"
#include "mathutil.h"
#include "fs-i6.h"
#include "ringlight.h"
#include "calibration.h"

#include "UnitStepper.h"

UnitStepper pitchStepper(PITCH_STEPPER_STEP, PITCH_STEPPER_DIR, 16, 0.44, 0, 90);
UnitStepper yawStepper(YAW_STEPPER_STEP, YAW_STEPPER_DIR, 16, 0.36, YAW_ZERO_OFFSET, 198);
UnitStepper zStepper(Z_STEPPER_STEP, Z_STEPPER_DIR, 8, 0.04078, 0, 75);
UnitStepper ringStepper(RING_STEPPER_STEP, RING_STEPPER_DIR, 32, 0.4, RING_ZERO_OFFSET, 239);
UnitStepper pipetteStepper(PIPETTE_STEPPER_STEP, PIPETTE_STEPPER_DIR, 32, 2.74, 100, 700);

unsigned long lastControlUpdate;
unsigned long lastDataUpdate;

void setup() {
	Serial.begin(1000000);
	// Serial.begin(115200);

	pinMode(BUTTON_A, INPUT);
	pinMode(SWITCH_A, INPUT);
	pinMode(SWITCH_B, INPUT);

	pinMode(PITCH_LIMIT_SWITCH, INPUT);
	pinMode(YAW_LIMIT_SWITCH, INPUT);
	pinMode(Z_LIMIT_SWITCH, INPUT);
	pinMode(RING_LIMIT_SWITCH, INPUT);
	pinMode(PIPETTE_LIMIT_SWITCH, INPUT);
	pinMode(BOWL_LIMIT_SWITCH, INPUT);

	pinMode(V12_CURRENT, INPUT);
	pinMode(V5_CURRENT, INPUT);

	// init 8-bank relay to HIGH (HIGH = off)
	InitPin(WATER_VALVE_RELAY, HIGH);
	InitPin(MILK_VALVE_RELAY, HIGH);
	InitPin(AIR_VALVE_RELAY, HIGH);
	InitPin(DRAINAGE_VALVE_RELAY, HIGH);
	InitPin(V12_RELAY_PIN1, HIGH);
	InitPin(V12_RELAY_PIN2, HIGH);
	InitPin(V5_RELAY_PIN, HIGH);

	// make steppers sleep on start
	InitPin(STEPPER_SLEEP, LOW);

	// Light pins
	InitPin(LIGHT_TOGGLE, HIGH);
	InitPin(LIGHT_MODE, HIGH);
	InitPin(LIGHT_RGB, HIGH);

	InitPin(STEP_INDICATOR_PIN, LOW);

	pitchStepper.setMaxSpeed(1250);
	pitchStepper.setAcceleration(1600);
	pitchStepper.setPinsInverted(true);

	yawStepper.setMaxSpeed(1250);
	yawStepper.setAcceleration(1600);
	yawStepper.setPinsInverted(true);

	zStepper.setMaxSpeed(1250);
	zStepper.setAcceleration(800);

	ringStepper.setPinsInverted(true);
	ringStepper.setMaxSpeed(1250);
	ringStepper.setAcceleration(800);

	pipetteStepper.setMaxSpeed(1250);
	pipetteStepper.setAcceleration(800);
	pipetteStepper.setPinsInverted(true);

	// Turn on 5V
	SetDualRelay(V5_RELAY_PIN, true);
	delay(100);
	RingLight::Toggle();

	Logger::SetLevel(Logger::VERBOSE);
}

void dataUpdate() {
	Serial.println();
	// Board input
	// Logger::PrintDataEntry("S_A", String(digitalRead(SWITCH_A)));
	// Logger::PrintDataEntry("S_B", String(digitalRead(SWITCH_B)));
	// Logger::PrintDataEntry("B_A", String(digitalRead(BUTTON_A)));
	
	// Limit switches
	// Logger::PrintDataEntry("P_LS", String(digitalRead(PITCH_LIMIT_SWITCH)));
	// Logger::PrintDataEntry("Y_LS", String(digitalRead(YAW_LIMIT_SWITCH)));
	// Logger::PrintDataEntry("Z_LS", String(digitalRead(Z_LIMIT_SWITCH)));
	// Logger::PrintDataEntry("R_LS", String(digitalRead(RING_LIMIT_SWITCH)));
	// Logger::PrintDataEntry("PIP_LS", String(digitalRead(PIPETTE_LIMIT_SWITCH)));
	// Logger::PrintDataEntry("BWL_LS", String(digitalRead(BOWL_LIMIT_SWITCH)));

	// Power
	// Logger::PrintDataEntry("V12_C", String(analogRead(V12_CURRENT)));
	// Logger::PrintDataEntry("V5_C", String(analogRead(V5_CURRENT)));

	// RX Controller data
	// FS_I6::PrintRawChannels();
	// FS_I6::PrintProcessedChannels();

	// stepper raw position
	Logger::PrintDataEntry("R_POS", String(ringStepper.currentPosition()));
	Logger::PrintDataEntry("Z_POS", String(zStepper.currentPosition()));
	Logger::PrintDataEntry("Y_POS", String(yawStepper.currentPosition()));
	Logger::PrintDataEntry("P_POS", String(pitchStepper.currentPosition()));
	Logger::PrintDataEntry("PP_POS", String(pipetteStepper.currentPosition()));

	// stepper units
	Logger::PrintDataEntry("R_UNIT", String(ringStepper.PositionToUnit(ringStepper.currentPosition())));
	Logger::PrintDataEntry("Z_UNIT", String(zStepper.PositionToUnit(zStepper.currentPosition())));
	Logger::PrintDataEntry("Y_UNIT", String(yawStepper.PositionToUnit(yawStepper.currentPosition())));
	Logger::PrintDataEntry("P_UNIT", String(pitchStepper.PositionToUnit(pitchStepper.currentPosition())));
	Logger::PrintDataEntry("PP_UNIT", String(pipetteStepper.PositionToUnit(pipetteStepper.currentPosition())));
}

bool ringAngleValid(double a) {
	return a >= ringStepper.GetMinUnit() and a <= ringStepper.GetMaxUnit();
}

bool goToIKPosition() {
	//todo: implement non-blocking goto ik logic
	return false;
}

bool goToPipettePosition() {
	//todo: implement non-blocking goto pipette logic
	return false;
}

void boundXYToCircle(float *x, float *y, float radius) {
	float mag = (float) hypotf((float) *x, (float) *y);
	if (mag > radius) {
		*x = *x / mag * radius;
		*y = *y / mag * radius;
	}
}

void updateRingAndYaw(float x, float y, float lastRing, float *ring, float *yaw) {
	boundXYToCircle(&x, &y, 0.7);
	// No solutions at centre so just do ring = last, yawOffset = 0
	if (abs(x) <= 0.001 && abs(y) <= 0.001) {
		*yaw = 0;
		*ring = lastRing;
		Logger::Debug("Centre point, so yawOffset = 0 and ring = "+String(lastRing)+" (last value)");
		return;
	}

	Logger::Debug("Target: " + String(x) + ", " + String(y));

	float x_mm = x * STAGE_RADIUS_MM;
	float y_mm = y * STAGE_RADIUS_MM;

	double xi, yi, xi_prime, yi_prime;

	int case_ = CircleCircleIntersection(
		0, 0, ARM_PATH_RADIUS_MM,
		x_mm, y_mm, ARM_PATH_RADIUS_MM,
		&xi, &yi, &xi_prime, &yi_prime
	);

	if (case_ == 0) {
		Logger::Warn("No solutions to circle intersection! Invalid calibration?");
		return;
	}

	// Now we have the 2 intersect points, and target x_mm, y_mm

	double angle = fmod(-atan2(xi, yi) * 180.0 / M_PI + 270.0, 360.0);
	double angle_prime = fmod(-atan2(xi_prime, yi_prime) * 180.0 / M_PI + 270.0, 360.0);

	Logger::Debug("angle="+String(angle)+" angle_prime="+String(angle_prime));

	bool use_i_prime = false;
	if (ringAngleValid(angle) && ringAngleValid(angle_prime)) {
		// move to whichever is closest to previous position
		if (abs(angle - lastRing) < abs(angle_prime - lastRing)) {
			*ring = (float) angle;
		} else {
			use_i_prime = true;
			*ring = (float) angle_prime;
		}
	} else if (ringAngleValid(angle)) {
		*ring = (float) angle;
	} else if (ringAngleValid(angle_prime)) {
		use_i_prime = true;
		*ring = (float) angle_prime;
	} else {
		*ring = (float) 5.0;
		Logger::Warn("Both intersection angles (" +
			String(angle) + ", " + String(angle_prime) +
			") are out of ring angle bounds. Setting to 5deg"
		);
	};

	Logger::Debug(use_i_prime ? "Chose angle_prime":"Chose angle");

	double newYaw;
	if (use_i_prime) {
		newYaw = -AngleBetweenVectors(-xi_prime, -yi_prime, x_mm - xi_prime, y_mm - yi_prime);
	} else {
		newYaw = -AngleBetweenVectors(-xi, -yi, x_mm - xi, y_mm - yi);
	}
	*yaw = (float) newYaw;
	Logger::Debug("Set ring=" + String(*ring) + " and newYaw=" + String(newYaw));
}

// ikModeUpdate does ik logic on an x and y in range (-1,1).
void ikModeUpdate() {
	//! write this
	// bool inPosition = goToIKPosition();
	// if (!inPosition) {
	// 	return;
	// }

	float x = FS_I6::GetStick(FS_I6::RH);
	float y = FS_I6::GetStick(FS_I6::RV);

	float ring, yaw;
	updateRingAndYaw(x, y,
		ringStepper.PositionToUnit(ringStepper.currentPosition()),
		&ring, &yaw
	);
	
	if (ring < ringStepper.GetMinUnit() || ring > ringStepper.GetMaxUnit()) {
		Logger::Warn("Unexpected ring value " + String(ring) + " detected, aborting ik!");
		return;
	}
	if (yaw < -20 || yaw > 20) {
		Logger::Warn("Potentially dangerous yaw value " + String(yaw) + " detected, aborting ik!");
		return;
	}

	Logger::Debug("Current ring = " + String(ringStepper.PositionToUnit(ringStepper.currentPosition())) + ", current yaw = " + String(yawStepper.PositionToUnit(yawStepper.currentPosition())));
	Logger::Debug("Target ring = " + String(ring) + ", final yaw = " + String(yaw));
	if (ENABLE_IK_ACTUATION) {
		ringStepper.moveTo(ringStepper.UnitToPosition(ring));
		yawStepper.moveTo(yawStepper.UnitToPosition(yaw));
	}
}

// pippetteModeUpdate does pipette logic with 
void pipetteModeUpdate() {
	bool inPosition = goToPipettePosition();
	if (!inPosition) {
		return;
	}

	//todo pipette logic
}

void controlUpdate() {
	// Get inputs

	int sw1 = FS_I6::GetSwitch(FS_I6::S1);
	int sw2 = FS_I6::GetSwitch(FS_I6::S2);

	bool boardSwitchA = digitalRead(SWITCH_A);

	// Sleep

	if (sw1) {
		Sleep::Wake();
	}

	// Power

	SetDualRelay(V12_RELAY_PIN1, sw1);
	SetDualRelay(V12_RELAY_PIN2, sw1);
	digitalWrite(STEPPER_SLEEP, sw1 ? HIGH : LOW);

	// Nothing to do if 12V is off.
	if (!sw1) return;

	// Main control
	// bool assistedMode = boardSwitchA;

	float speedMult = 1600.0;

	if (sw2 == 0 || sw2 == 1) {
		// manual + pipette
		// speed <> position bug workaround
		ringStepper.stop();
		yawStepper.stop();
		pitchStepper.stop();
		pipetteStepper.stop();

		float left_h = FS_I6::GetStick(FS_I6::LH);
		if (sw2 == 0) {
			ringStepper.setSpeed(speedMult*left_h);
		} else if (sw2 == 1) {
			pipetteStepper.setSpeed(-speedMult*left_h);
		}
		float right_h = FS_I6::GetStick(FS_I6::RH);
		yawStepper.setSpeed(speedMult*right_h);
		float right_v = FS_I6::GetStick(FS_I6::RV);
		pitchStepper.setSpeed(speedMult*right_v);
		zStepper.SetMinUnit(0);
	} else if (sw2 == 2) {
		// ik
		// block mode if pitch is low
		if (pitchStepper.PositionToUnit(pitchStepper.currentPosition()) >= CENTRE_PITCH - 1 &&
			zStepper.PositionToUnit(zStepper.currentPosition()) >= MIN_BOWL_Z) {
			ikModeUpdate();

			float left_h = FS_I6::GetStick(FS_I6::LH);
			pipetteStepper.setSpeed(-speedMult*left_h);

			pitchStepper.moveTo(pitchStepper.UnitToPosition(CENTRE_PITCH));
			zStepper.SetMinUnit(MIN_BOWL_Z);
		}
	}

	float left_v = FS_I6::GetStick(FS_I6::LV);
	zStepper.setSpeed(speedMult*left_v);
}

//todo: move the following 4 functions to inside UnitStepper

void processLimitSwitch(uint8_t limitSw, UnitStepper *stepper) {
	// if limit switch is pressed, reset position and only allow positive speeds
	if (digitalRead(limitSw)) {
		float oldSpeed = stepper->speed();
		stepper->setCurrentPosition(stepper->UnitToPosition(stepper->GetMinUnit()));
		if (oldSpeed > 0) {
			stepper->setSpeed(oldSpeed);
		}
		stepper->MarkAsCalibrated();
	}
}

void processStepperLimits(UnitStepper *stepper) {
	if (!stepper->IsCalibrated()) {
		// Limits are meaningless without known position
		return;
	}

	// Block speeds if over max position
	if (stepper->PositionToUnit(stepper->currentPosition()) >= stepper->GetMaxUnit()) {
		if (stepper->speed() > 0) {
			stepper->setSpeed(0);
		}
	}
	// Block speeds if under min position
	if (stepper->PositionToUnit(stepper->currentPosition()) <= stepper->GetMinUnit()) {
		if (stepper->speed() < 0) {
			stepper->setSpeed(0);
		}
	}
}

void limitSwitchUpdate() {
	processLimitSwitch(RING_LIMIT_SWITCH, &ringStepper);
	processLimitSwitch(Z_LIMIT_SWITCH, &zStepper);
	processLimitSwitch(YAW_LIMIT_SWITCH, &yawStepper);
	processLimitSwitch(PITCH_LIMIT_SWITCH, &pitchStepper);
	processLimitSwitch(PIPETTE_LIMIT_SWITCH, &pipetteStepper);
}

void stepperLimits() {
	processStepperLimits(&ringStepper);
	processStepperLimits(&zStepper);
	processStepperLimits(&yawStepper);
	processStepperLimits(&pitchStepper);
	processStepperLimits(&pipetteStepper);
}

//end todo for unitstepper refactor

void runSteppers() {
	digitalWrite(STEP_INDICATOR_PIN, HIGH);
	// seriously sort this out
	int sw2 = FS_I6::GetSwitch(FS_I6::S2);
	if (sw2 == 0 || sw2 == 1) {
		// manual
		ringStepper.runSpeed();
		pitchStepper.runSpeed();
		yawStepper.runSpeed();
		zStepper.runSpeed();
		pipetteStepper.runSpeed();
	}
	if (sw2 == 2) {
		// ik
		ringStepper.run();
		pitchStepper.run();
		yawStepper.run();
		zStepper.runSpeed();
		pipetteStepper.runSpeed();
	}
	digitalWrite(STEP_INDICATOR_PIN, LOW);
}

int updatesInLastSecond;
unsigned long lastUpdatesPerSecondTime = millis();
int updatesPerSecond;

void loop() {
	Sleep::Update();
	if (Sleep::IsSleeping()) {
		delay(50);
		return;
	}

	//? Maybe cache input

	// control
	if (millis() - lastControlUpdate > 100) {
		controlUpdate();
		lastControlUpdate = millis();
	}

	// Called after because it will prevent invalid movement.
	//todo: integrate to UnitStepper
	limitSwitchUpdate();

	// Bounding / collision detection
	//? Maybe this should be a check in controller so it's aware of the limits
	//todo: integrate to UnitStepper
	stepperLimits();

	// actuation 
	runSteppers();

	if (PRINT_DATA && millis() - lastDataUpdate > 1000) {
		unsigned long now = millis();
		dataUpdate();
		Logger::PrintDataEntry("DATA_MS", String(millis() - now));
		Logger::PrintDataEntry("UPS", String(updatesPerSecond));
		Serial.println();
		lastDataUpdate = millis();
	}

	updatesInLastSecond++;
	if (millis() - lastUpdatesPerSecondTime > 1000) {
		updatesPerSecond = updatesInLastSecond;
		updatesInLastSecond = 0;
		lastUpdatesPerSecondTime = millis();
	}
}