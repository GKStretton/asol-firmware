#include <AccelStepper.h>
#include <Wire.h>
#include "src/common/ik_algorithm.h"
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
#include "src/drivers/i2c_eeprom.h"
#include "src/app/state_report.h"

State s = {
	updatesPerSecond: 0,
	lastNode: HOME,
	localTargetNode: UNDEFINED,
	globalTargetNode: HOME,
	manualRequested: false,
	lastControlUpdate: 0,
	lastDataUpdate: 0,
	pitchStepper: UnitStepper(PITCH_STEPPER_STEP, PITCH_STEPPER_DIR, 16, 0.44, -2.5, 90),
	yawStepper: UnitStepper(YAW_STEPPER_STEP, YAW_STEPPER_DIR, 8, 0.36, YAW_ZERO_OFFSET, 198),
	zStepper: UnitStepper(Z_STEPPER_STEP, Z_STEPPER_DIR, 4, 0.04078, 1, 73),
	ringStepper: UnitStepper(RING_STEPPER_STEP, RING_STEPPER_DIR, 32, 0.4, RING_ZERO_OFFSET, 215),
	pipetteStepper: UnitStepper(PIPETTE_STEPPER_STEP, PIPETTE_STEPPER_DIR, 32, 2.74, 0, 600),
	target_x: 0.0,
	target_y: 0.0,
	target_ring: RING_ZERO_OFFSET,
	target_yaw: 0.0,
	collectionRequest: {true, 0, 0, 0.0},
	pipetteState: {true, 0, 0.0},
	collectionInProgress: false,
	shutdownRequested: false,
	calibrationCleared: false,
	postCalibrationStopCalled: false,
	forceIdleLocation: true,
};

Controller controller;

int updatesInLastSecond;
unsigned long lastUpdatesPerSecondTime = millis();

// eepromStartup reads the startup counter, increments it, writes and prints it.
void eepromStartup() {
	uint8_t counter = I2C_EEPROM::ReadByte(STARTUP_COUNTER_MEM_ADDR);
	counter++;
	I2C_EEPROM::WriteByte(STARTUP_COUNTER_MEM_ADDR, counter);
	Logger::Info("Startup counter incremented to " + String(counter));
}

void sleepHandler(Sleep::SleepStatus sleepStatus) {
	digitalWrite(STEPPER_SLEEP, LOW);

	if (Sleep::IsEStopActive()) {
		StateReport_SetStatus(machine_Status_E_STOP_ACTIVE);
	} else {
		StateReport_SetStatus(machine_Status_SLEEPING);
	}
	StateReport_Update(&s);
}

void wakeHandler(Sleep::SleepStatus lastSleepStatus) {
	StateReport_SetStatus(machine_Status_WAKING_UP);
	StateReport_Update(&s);

	RingLight::Toggle();
	s.ClearState();
}

void setup()
{

	Wire.begin();
	Serial.begin(1000000);
	Logger::SetLevel(Logger::DEBUG);
	Logger::Info("setup start");

	pinMode(E_STOP_PIN, INPUT);

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
	InitPin(FRONT_LIGHT_TOGGLE, HIGH);
	InitPin(TOP_LIGHT_TOGGLE, HIGH);
	InitPin(TOP_LIGHT_MODE, HIGH);
	InitPin(TOP_LIGHT_RGB, HIGH);

	InitPin(STEP_INDICATOR_PIN, LOW);

	initSteppers();

	// register callback
	SerialMQTT::SetTopicHandler(topicHandler);

	Sleep::SetOnWakeHandler(wakeHandler);
	Sleep::SetOnSleepHandler(sleepHandler);

	// disabling so it doesn't always start after flash / gateway restart
	// Sleep::Wake();

	eepromStartup();

	Logger::Info("Sending first state report");
	StateReport_SetStatus(machine_Status_SLEEPING);
	StateReport_Update(&s);
	Logger::Info("setup complete");
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
		Sleep::Sleep(Sleep::UNKNOWN);
	}
	else if (topic == "mega/req/shutdown")
	{
		s.shutdownRequested = true;
	}
	else if (topic == "mega/req/uncalibrate")
	{
		s.pitchStepper.MarkAsNotCalibrated();
		s.yawStepper.MarkAsNotCalibrated();
		s.zStepper.MarkAsNotCalibrated();
		s.ringStepper.MarkAsNotCalibrated();
		s.pipetteStepper.MarkAsNotCalibrated();
		s.calibrationCleared = true;
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
		SerialMQTT::UnpackCommaSeparatedValues(payload, values, 1);
		float ul = values[0].toFloat();
		if (!s.pipetteState.spent) {
			s.pipetteState.ulVolumeHeldTarget -= ul;
			if (s.pipetteState.ulVolumeHeldTarget <= 0) {
				s.pipetteState.ulVolumeHeldTarget = 0;
			}
			Logger::Info("dispensed " + String(ul) + ", ulVolumeHeldTarget is now " + String(s.pipetteState.ulVolumeHeldTarget));
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
		s.forceIdleLocation = num == IDLE_LOCATION;
		s.SetGlobalNavigationTarget((Node)num);
		Logger::Debug("Set globalTargetNode to " + String(num));
	}
	else if (topic == "mega/req/goto-xy") {
		String values[] = {"", ""};
		SerialMQTT::UnpackCommaSeparatedValues(payload, values, 2);
		float target_x = values[0].toFloat();
		float target_y = values[1].toFloat();
		Logger::Info("recieved req for target_x, target_y to " + String(target_x) + ", " + String(target_y));

		float ring, yaw;
		int code = getRingAndYawFromXY(target_x, target_y,
						s.ringStepper.PositionToUnit(s.ringStepper.currentPosition()),
						&ring, &yaw,
						s.ringStepper.GetMinUnit(), s.ringStepper.GetMaxUnit());
		
		if (code != 0) {
			Logger::Error("error code fromgetRingAndYawFromXY, aborting");
			return;
		}

		if (ring < s.ringStepper.GetMinUnit() || ring > s.ringStepper.GetMaxUnit()) {
			Logger::Error("Unexpected ring value " + String(ring) + " detected, aborting ik!");
			return;
		}
		if (yaw < -MAX_BOWL_YAW || yaw > MAX_BOWL_YAW) {
			Logger::Error("Potentially dangerous yaw value " + String(yaw) + " detected, aborting ik!");
			return;
		}
		Logger::Info("Setting x,y, and target_ring=" + String(ring) + " and target_yaw=" + String(yaw));
		s.target_x = target_x;
		s.target_y = target_y;
		s.target_ring = ring;
		s.target_yaw = yaw;
	}
	else if (topic == "mega/req/manual")
	{
		s.manualRequested = !s.manualRequested;
		Logger::Info("Toggled manualRequested mode to " + String(s.manualRequested));
	}
	else if (topic == "mega/req/state-report")
	{
		StateReport_ForceSend();
	}
	else if (topic == "mega/req/pin-on")
	{
		uint8_t pin = (uint8_t) payload.toInt();
		digitalWrite(pin, HIGH);
	}
	else if (topic == "mega/req/pin-off")
	{
		uint8_t pin = (uint8_t) payload.toInt();
		digitalWrite(pin, LOW);
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
	// SerialMQTT::PublishMega("d/S_A", String(digitalRead(SWITCH_A)));
	// SerialMQTT::PublishMega("d/S_B", String(digitalRead(SWITCH_B)));
	// SerialMQTT::PublishMega("d/B_A", String(digitalRead(BUTTON_A)));

	// Power
	// SerialMQTT::PublishMega("d/V12_C", String(analogRead(V12_CURRENT)));
	// SerialMQTT::PublishMega("d/V5_C", String(analogRead(V5_CURRENT)));

	// RX Controller data
	// FS_I6::PrintRawChannels();
	// FS_I6::PrintProcessedChannels();

	// stepper raw position
	// SerialMQTT::PublishMega("d/R_POS", String(s.ringStepper.currentPosition()));
	// SerialMQTT::PublishMega("d/Z_POS", String(s.zStepper.currentPosition()));
	SerialMQTT::PublishMega("d/Y_POS", String(s.yawStepper.currentPosition()));
	// SerialMQTT::PublishMega("d/P_POS", String(s.pitchStepper.currentPosition()));
	// SerialMQTT::PublishMega("d/PP_POS", String(s.pipetteStepper.currentPosition()));

	// stepper units
	// SerialMQTT::PublishMega("d/R_UNIT", String(s.ringStepper.PositionToUnit(s.ringStepper.currentPosition())));
	// SerialMQTT::PublishMega("d/Z_UNIT", String(s.zStepper.PositionToUnit(s.zStepper.currentPosition())));
	SerialMQTT::PublishMega("d/Y_UNIT", String(s.yawStepper.PositionToUnit(s.yawStepper.currentPosition())));
	// SerialMQTT::PublishMega("d/P_UNIT", String(s.pitchStepper.PositionToUnit(s.pitchStepper.currentPosition())));
	// SerialMQTT::PublishMega("d/PP_UNIT", String(s.pipetteStepper.PositionToUnit(s.pipetteStepper.currentPosition())));

	// SerialMQTT::PublishMega("d/PP_L_SW", String(digitalRead(PIPETTE_LIMIT_SWITCH)));
	// SerialMQTT::PublishMega("d/PP_CALI", String(s.pipetteStepper.IsCalibrated()));

	// SerialMQTT::PublishMega("d/DATA_MS", String(millis() - start));
	// SerialMQTT::PublishMega("d/UPS", String(s.updatesPerSecond));
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
		delay(50);
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
