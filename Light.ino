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
#include "serialmqtt.h"

#include "UnitStepper.h"

UnitStepper pitchStepper(PITCH_STEPPER_STEP, PITCH_STEPPER_DIR, 16, 0.44, 0, 90);
UnitStepper yawStepper(YAW_STEPPER_STEP, YAW_STEPPER_DIR, 8, 0.36, YAW_ZERO_OFFSET, 198);
UnitStepper zStepper(Z_STEPPER_STEP, Z_STEPPER_DIR, 4, 0.04078, 0, 73);
UnitStepper ringStepper(RING_STEPPER_STEP, RING_STEPPER_DIR, 32, 0.4, RING_ZERO_OFFSET, 280);
UnitStepper pipetteStepper(PIPETTE_STEPPER_STEP, PIPETTE_STEPPER_DIR, 32, 2.74, 100, 700);

unsigned long lastControlUpdate;
unsigned long lastDataUpdate;


void setup() {
	Serial.begin(1000000);
	Logger::SetLevel(Logger::DEBUG);
	Logger::Info("setup start");

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


	Sleep::Wake();

	// register callback
	SerialMQTT::SetTopicHandler(topicHandler);
	Logger::Info("setup complete");
}

void topicHandler(String topic, String payload) {
	if (topic == "mega/req/wake") {
		Sleep::Wake();
		return;
	}
	if (Sleep::IsSleeping()) {
		// only listen for wake if asleep
		return;
	}

	if (topic == "mega/req/sleep") {
		Sleep::Sleep();
	} else if (topic == "mega/req/uncalibrate") {
		pitchStepper.MarkAsNotCalibrated();
		yawStepper.MarkAsNotCalibrated();
		zStepper.MarkAsNotCalibrated();
		ringStepper.MarkAsNotCalibrated();
		pipetteStepper.MarkAsNotCalibrated();
	} else if (topic == "mega/req/open-drain") {
		SetDualRelay(DRAINAGE_VALVE_RELAY, true);
		//todo replace these with topic events
		Logger::Info("draining...");
	} else if (topic == "mega/req/close-drain") {
		SetDualRelay(DRAINAGE_VALVE_RELAY, false);
		Logger::Info("closing drain.");
	// } else if (topic == "mega/req/goto-xy") {
		// todo: unpack payload into ints
		// Logger::Debug("goto-xy ACK:" + String(x_target) + String(y_target))
	// } else if (topic == "mega/req/dispense") {
	// } else if (topic == "mega/req/collect") {
	} else {
		Logger::Debug("no handler for " + topic + " (payload = " + payload + ")");
	}
}

void dataUpdate() {
	Serial.println();
	// Board input
	// SerialMQTT::Publish("d/S_A", String(digitalRead(SWITCH_A)));
	// SerialMQTT::Publish("d/S_B", String(digitalRead(SWITCH_B)));
	// SerialMQTT::Publish("d/B_A", String(digitalRead(BUTTON_A)));
	
	// Limit switches
	// SerialMQTT::Publish("d/P_LS", String(digitalRead(PITCH_LIMIT_SWITCH)));
	// SerialMQTT::Publish("d/Y_LS", String(digitalRead(YAW_LIMIT_SWITCH)));
	// SerialMQTT::Publish("d/Z_LS", String(digitalRead(Z_LIMIT_SWITCH)));
	// SerialMQTT::Publish("d/R_LS", String(digitalRead(RING_LIMIT_SWITCH)));
	// SerialMQTT::Publish("d/PIP_LS", String(digitalRead(PIPETTE_LIMIT_SWITCH)));
	// SerialMQTT::Publish("d/BWL_LS", String(digitalRead(BOWL_LIMIT_SWITCH)));

	// Power
	// SerialMQTT::Publish("d/V12_C", String(analogRead(V12_CURRENT)));
	// SerialMQTT::Publish("d/V5_C", String(analogRead(V5_CURRENT)));

	// RX Controller data
	FS_I6::PrintRawChannels();
	FS_I6::PrintProcessedChannels();

	// stepper raw position
	SerialMQTT::Publish("d/R_POS", String(ringStepper.currentPosition()));
	SerialMQTT::Publish("d/Z_POS", String(zStepper.currentPosition()));
	SerialMQTT::Publish("d/Y_POS", String(yawStepper.currentPosition()));
	SerialMQTT::Publish("d/P_POS", String(pitchStepper.currentPosition()));
	SerialMQTT::Publish("d/PP_POS", String(pipetteStepper.currentPosition()));

	// stepper units
	SerialMQTT::Publish("d/R_UNIT", String(ringStepper.PositionToUnit(ringStepper.currentPosition())));
	SerialMQTT::Publish("d/Z_UNIT", String(zStepper.PositionToUnit(zStepper.currentPosition())));
	SerialMQTT::Publish("d/Y_UNIT", String(yawStepper.PositionToUnit(yawStepper.currentPosition())));
	SerialMQTT::Publish("d/P_UNIT", String(pitchStepper.PositionToUnit(pitchStepper.currentPosition())));
	SerialMQTT::Publish("d/PP_UNIT", String(pipetteStepper.PositionToUnit(pipetteStepper.currentPosition())));
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

float x_target = 0;
float y_target = 0;

// ikModeUpdate does ik logic on an x and y in range (-1,1).
void ikModeUpdate() {
	//! write this
	// bool inPosition = goToIKPosition();
	// if (!inPosition) {
	// 	return;
	// }

	if (FS_I6::GetSwitch(FS_I6::S2) == 2) {
		float dx = FS_I6::GetStick(FS_I6::RH);
		float dy = FS_I6::GetStick(FS_I6::RV);

		float sf = 0.05;
		x_target += sf * dx;
		y_target += sf * dy;
	}

	float ring, yaw;
	updateRingAndYaw(x_target, y_target,
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

	digitalWrite(STEPPER_SLEEP, sw1 ? HIGH : LOW);

	// Nothing to do if steppers asleep
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

/*
handlers can use this to parse payload:
bool(err) splitString(args**) {

}
*/

int updatesInLastSecond;
unsigned long lastUpdatesPerSecondTime = millis();
int updatesPerSecond;

void loop() {
	SerialMQTT::Update();

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
	//? Maybe this shold be a check in controller so it's aware of the limits
	//todo: integrate to UnitStepper
	stepperLimits();

	// actuation 
	runSteppers();

	if (PRINT_DATA && millis() - lastDataUpdate > 1000) {
		unsigned long now = millis();
		dataUpdate();
		SerialMQTT::Publish("d/DATA_MS", String(millis() - now));
		SerialMQTT::Publish("d/UPS", String(updatesPerSecond));
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