#include <AccelStepper.h>
#include "src/drivers/fluid.h"
#include "src/drivers/fs-i6.h"
#include "src/config.h"
#include "src/middleware/logger.h"
#include "src/middleware/sleep.h"
#include "src/common/util.h"
#include "src/common/mathutil.h"
#include "src/drivers/ringlight.h"
#include "src/calibration.h"
#include "src/middleware/serialmqtt.h"
#include "src/drivers/UnitStepper.h"
#include "src/app/state.h"
#include "src/app/controller.h"

State s = {
	0,
	HOME,
	UNDEFINED,
	HOME,
	false,
	0,
	0,
	UnitStepper(PITCH_STEPPER_STEP, PITCH_STEPPER_DIR, 16, 0.44, 0, 90),
	UnitStepper(YAW_STEPPER_STEP, YAW_STEPPER_DIR, 8, 0.36, YAW_ZERO_OFFSET, 198),
	UnitStepper(Z_STEPPER_STEP, Z_STEPPER_DIR, 4, 0.04078, 0, 73),
	UnitStepper(RING_STEPPER_STEP, RING_STEPPER_DIR, 32, 0.4, RING_ZERO_OFFSET, 280),
	UnitStepper(PIPETTE_STEPPER_STEP, PIPETTE_STEPPER_DIR, 32, 2.74, 100, 700),
	0.0,
	0.0,
	{true, 0, 0, 0.0},
	{true, 0, 0.0},
	false,
};

Controller controller;

int updatesInLastSecond;
unsigned long lastUpdatesPerSecondTime = millis();

void setup()
{
	pinMode(E_STOP_PIN, INPUT);

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

	initSteppers();

	// register callback
	SerialMQTT::SetTopicHandler(topicHandler);
	Logger::Info("setup complete");

	Sleep::Wake();
}

void initSteppers() {
	s.pitchStepper.setMaxSpeed(1250 * SPEED_MULT);
	s.pitchStepper.setAcceleration(1600 * SPEED_MULT);
	s.pitchStepper.setPinsInverted(true);
	s.pitchStepper.SetLimitSwitchPin(PITCH_LIMIT_SWITCH);
	s.pitchStepper.SetAtTargetUnitThreshold(0);

	s.yawStepper.setMaxSpeed(1250 * SPEED_MULT);
	s.yawStepper.setAcceleration(1600 * SPEED_MULT);
	s.yawStepper.setPinsInverted(true);
	s.yawStepper.SetLimitSwitchPin(YAW_LIMIT_SWITCH);
	s.yawStepper.SetAtTargetUnitThreshold(0);

	s.zStepper.setMaxSpeed(1250 * SPEED_MULT);
	s.zStepper.setAcceleration(800 * SPEED_MULT);
	s.zStepper.SetLimitSwitchPin(Z_LIMIT_SWITCH);
	s.zStepper.SetAtTargetUnitThreshold(0);

	s.ringStepper.setPinsInverted(true);
	s.ringStepper.setMaxSpeed(1250 * SPEED_MULT);
	s.ringStepper.setAcceleration(800 * SPEED_MULT);
	s.ringStepper.SetLimitSwitchPin(RING_LIMIT_SWITCH);

	s.pipetteStepper.setMaxSpeed(1250 * SPEED_MULT);
	s.pipetteStepper.setAcceleration(800 * SPEED_MULT);
	s.pipetteStepper.setPinsInverted(true);
	s.pipetteStepper.SetLimitSwitchPin(PIPETTE_LIMIT_SWITCH);
}

void topicHandler(String topic, String payload)
{
	Logger::Debug("topic handler start");
	if (topic == "mega/req/wake")
	{
		Sleep::Wake();
		return;
	}
	if (Sleep::IsSleeping())
	{
		// only listen for wake if asleep
		return;
	}

	if (topic == "mega/req/sleep")
	{
		Sleep::Sleep();
	}
	else if (topic == "mega/req/uncalibrate")
	{
		s.pitchStepper.MarkAsNotCalibrated();
		s.yawStepper.MarkAsNotCalibrated();
		s.zStepper.MarkAsNotCalibrated();
		s.ringStepper.MarkAsNotCalibrated();
		s.pipetteStepper.MarkAsNotCalibrated();
	}
	else if (topic == "mega/req/open-drain")
	{
		SetDualRelay(DRAINAGE_VALVE_RELAY, true);
		// todo replace these with topic events
		Logger::Info("draining...");
	}
	else if (topic == "mega/req/close-drain")
	{
		SetDualRelay(DRAINAGE_VALVE_RELAY, false);
		Logger::Info("closing drain.");
	}
	else if (topic == "mega/req/dispense") {
		String values[] = {""};
		float ul = values[0].toFloat();
		SerialMQTT::UnpackCommaSeparatedValues(payload, values, 1);
		if (!s.pipetteState.spent) {
			s.pipetteState.ulVolumeHeldTarget -= ul;
			if (s.pipetteState.ulVolumeHeldTarget <= 0) {
				s.pipetteState.ulVolumeHeldTarget = 0;
			}
			Logger::Info("dispensed, ulVolumeHeldTarget is now " + String(s.pipetteState.ulVolumeHeldTarget));
		} else {
			Logger::Info("Cannot dispense because already spent");
		}
	}
	else if (topic == "mega/req/collect") {
		String values[] = {"", ""};
		SerialMQTT::UnpackCommaSeparatedValues(payload, values, 2);
		int vial = values[0].toInt();
		float ul = values[1].toFloat();

		if (!s.collectionRequest.requestCompleted) {
			Logger::Info("cannot collect because collection request " + String(s.collectionRequest.requestNumber) + " is still in progress");
		} else {
			s.collectionRequest.requestNumber++;
			s.collectionRequest.requestCompleted = false;
			s.collectionRequest.vialNumber = vial;
			s.collectionRequest.ulVolume = ul;
			Logger::Info("created collection request " + String(s.collectionRequest.requestNumber) + " for " + String(ul) + "ul of vial " + String(vial));
		}
	}
	else if (topic == "mega/req/goto-node")
	{
		long num = payload.toInt();
		s.globalTargetNode = (Node)num;
		Logger::Debug("Set globalTargetNode to " + String(num));
	}
	else if (topic == "mega/req/goto-xy") {
		String values[] = {"", ""};
		SerialMQTT::UnpackCommaSeparatedValues(payload, values, 2);
		s.target_x = values[0].toFloat();
		s.target_y = values[1].toFloat();
		Logger::Info("set target_x, target_y to " + String(s.target_x) + ", " + String(s.target_y));
	}
	else
	{
		Logger::Debug("no handler for " + topic + " (payload = " + payload + ")");
	}
}

void dataUpdate()
{
	if (!PRINT_DATA) return;
	if (millis() - s.lastDataUpdate < 1000) return;
	s.lastDataUpdate = millis();

	unsigned long start = millis();
	// Board input
	// SerialMQTT::Publish("d/S_A", String(digitalRead(SWITCH_A)));
	// SerialMQTT::Publish("d/S_B", String(digitalRead(SWITCH_B)));
	// SerialMQTT::Publish("d/B_A", String(digitalRead(BUTTON_A)));

	// Power
	// SerialMQTT::Publish("d/V12_C", String(analogRead(V12_CURRENT)));
	// SerialMQTT::Publish("d/V5_C", String(analogRead(V5_CURRENT)));

	// RX Controller data
	// FS_I6::PrintRawChannels();
	// FS_I6::PrintProcessedChannels();

	// stepper raw position
	// SerialMQTT::Publish("d/R_POS", String(s.ringStepper.currentPosition()));
	SerialMQTT::Publish("d/Z_POS", String(s.zStepper.currentPosition()));
	SerialMQTT::Publish("d/Y_POS", String(s.yawStepper.currentPosition()));
	SerialMQTT::Publish("d/P_POS", String(s.pitchStepper.currentPosition()));
	// SerialMQTT::Publish("d/PP_POS", String(s.pipetteStepper.currentPosition()));

	// stepper units
	// SerialMQTT::Publish("d/R_UNIT", String(s.ringStepper.PositionToUnit(s.ringStepper.currentPosition())));
	SerialMQTT::Publish("d/Z_UNIT", String(s.zStepper.PositionToUnit(s.zStepper.currentPosition())));
	SerialMQTT::Publish("d/Y_UNIT", String(s.yawStepper.PositionToUnit(s.yawStepper.currentPosition())));
	SerialMQTT::Publish("d/P_UNIT", String(s.pitchStepper.PositionToUnit(s.pitchStepper.currentPosition())));
	// SerialMQTT::Publish("d/PP_UNIT", String(s.pipetteStepper.PositionToUnit(s.pipetteStepper.currentPosition())));

	SerialMQTT::Publish("d/DATA_MS", String(millis() - start));
	SerialMQTT::Publish("d/UPS", String(s.updatesPerSecond));
}

void runSteppers(State *s)
{
	digitalWrite(STEP_INDICATOR_PIN, HIGH);
	s->ringStepper.Update();
	s->pitchStepper.Update();
	s->yawStepper.Update();
	s->zStepper.Update();
	s->pipetteStepper.Update();
	digitalWrite(STEP_INDICATOR_PIN, LOW);
}

void loop()
{
	SerialMQTT::Update();

	Sleep::Update();
	if (Sleep::IsSleeping())
	{
		return;
	}

	//? Maybe cache input

	controller.Update(&s);

	// Run Steppers
	runSteppers(&s);

	dataUpdate();

	updatesInLastSecond++;
	if (millis() - lastUpdatesPerSecondTime > 1000)
	{
		s.updatesPerSecond = updatesInLastSecond;
		updatesInLastSecond = 0;
		lastUpdatesPerSecondTime = millis();
	}
}
