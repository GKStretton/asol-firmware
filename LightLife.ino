#include <simpleRPC.h>
#include "switch.h"
#include "stage.h"
#include "fluid.h"
#include "config.h"
#include "util.h"

#include <PPMReader.h>
#include <AccelStepper.h>

PPMReader ppm(CONTROLLER_PPM, PPM_CHANNELS);

AccelStepper pitchStepper(AccelStepper::DRIVER, PITCH_STEPPER_STEP, PITCH_STEPPER_DIR);
AccelStepper yawStepper(AccelStepper::DRIVER, YAW_STEPPER_STEP, YAW_STEPPER_DIR);
AccelStepper zStepper(AccelStepper::DRIVER, Z_STEPPER_STEP, Z_STEPPER_DIR);

unsigned long lastInputUpdate;

void setup() {
	Serial.begin(115200);

	// init 8-bank relay to HIGH (HIGH = off)
	InitPin(WATER_VALVE_RELAY, HIGH);
	InitPin(MILK_VALVE_RELAY, HIGH);
	InitPin(AIR_VALVE_RELAY, HIGH);
	InitPin(DRAINAGE_VALVE_RELAY, HIGH);
	InitPin(V12_RELAY_PIN1, HIGH);
	InitPin(V12_RELAY_PIN2, HIGH);

	// make steppers sleep on start
	InitPin(STEPPER_SLEEP, LOW);

	pitchStepper.setMaxSpeed(600);
	pitchStepper.setAcceleration(3000);

	yawStepper.setMaxSpeed(800);
	yawStepper.setAcceleration(3000);

	zStepper.setMaxSpeed(2000);
	zStepper.setAcceleration(5000);
}

void printRXChannelsLoop() {
	for (byte i = 1; i <= PPM_CHANNELS; i++) {
		unsigned int value = ppm.latestValidChannelValue(i, 0);
		Serial.print(String(value) + "\t");
	}
	Serial.println();
	delay(50);
}

float h_right;
float v_right;
float h_left;
float v_left;

void overrideLoop() {
	float pitch;
	if (millis() - lastInputUpdate > 10) {
		// switches on channel 5 and 6
		bool sw1 = ppm.latestValidChannelValue(5, 0) >= 1600 ? true : false;
		bool sw2 = ppm.latestValidChannelValue(6, 0) >= 1600 ? true : false;

		float alpha = 0.98;

		h_right = h_right * alpha + float((ppm.latestValidChannelValue(1, 1000)) - 1500.0) * (1.0 - alpha);
		v_right = v_right * alpha + float((ppm.latestValidChannelValue(2, 1000)) - 1500.0) * (1.0 - alpha);
		v_left = v_left * alpha + float((ppm.latestValidChannelValue(3, 1000)) - 1000.0) * (1.0 - alpha);
		h_left = h_left * alpha + float((ppm.latestValidChannelValue(4, 1000)) - 1500.0) * (1.0 - alpha);


		Serial.print(sw1 ? "on" : "off");
		Serial.print("\t");
		Serial.print(sw2 ? "on" : "off");
		Serial.print("\t");
		Serial.print(String(v_left) + "\t");
		Serial.print(String(h_left) + "\t");
		Serial.print(String(v_right) + "\t");
		Serial.print(String(h_right) + "\n");

		SetDualRelay(V12_RELAY_PIN1, sw1);
		SetDualRelay(V12_RELAY_PIN2, sw1);

		digitalWrite(STEPPER_SLEEP, sw2 ? HIGH : LOW);

		pitchStepper.moveTo(v_right);
		yawStepper.moveTo(-h_right);
		zStepper.moveTo(v_left * 4);

		lastInputUpdate = millis();
	}

	pitchStepper.run();
	yawStepper.run();
	zStepper.run();
}

void loop() {
	if (OVERRIDE_LOOP) {
		overrideLoop();
	} else {
		interface(
			Serial,

			SetCameraState, "SetCameraState: turns the DSLR on or off. @on: state",
			SetRingLightState, "SetRingLightState: turns the ring light on or off. @on: state",
			SetMainsState, "SetMainsState: switches the AC power input. @on: state",
			SetLED, "SetLED: Turn the on-board led on or off. @on: boolean on or off.",

			Drain, "Drain: Set drainage state. @drain: boolean for whether to be draining",

			SetAirFlow,   "SetAirFlow: Turn the flow of air or off in the tubing @flow: boolean for flow",
			SetFlushFlow,  "SetFlushFlow: Turn the dispension of the flushing fluid on or off @flow: boolean for flow",
			SetCanvasFlow, "SetCanvasFlow: Turn the dispension of the canvas fluid on or off @flow: boolean for flow"
		);
	}
}