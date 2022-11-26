#include "stage.h"
#include <Servo.h>
#include "../config.h"
#include "../common/util.h"

Servo drainageServo;

void setDrainageArmAngle(int angle) {
	Bound(&angle, DRAINAGE_REST_ANGLE, DRAINAGE_CONTACT_ANGLE);
	drainageServo.write(180 - angle);
}

void SetupStage() {
	drainageServo.attach(DRAINAGE_SERVO);
	setDrainageArmAngle(DRAINAGE_REST_ANGLE);

	// init high because dual relay
	InitPin(DRAINAGE_VALVE_RELAY, HIGH);

	/*
	InitPin(BOWL_STEPPER_DIR, LOW);
	InitPin(BOWL_STEPPER_STEP, LOW);
	InitPin(DISC_STEPPER_DIR, LOW);
	InitPin(DISC_STEPPER_STEP, LOW);
	*/
}

void Drain(bool drain) {
	//! goto / ensure in correct rotation

	if (drain) {
		setDrainageArmAngle(DRAINAGE_CONTACT_ANGLE);
		delay(500);
		// turn on after contact is made to prevent arcing.
		SetDualRelay(DRAINAGE_VALVE_RELAY, true);
	} else {
		// turn off first and wait a bit to prevent arcing.
		SetDualRelay(DRAINAGE_VALVE_RELAY, false);
		delay(200);
		setDrainageArmAngle(DRAINAGE_REST_ANGLE);
	}
}
