#include <AccelStepper.h>
#include <Wire.h>
#include "src/common/ik_algorithm.h"
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
#include "src/app/navigation.h"
#include "src/app/controller.h"
#include "src/drivers/i2c_eeprom.h"
#include "src/app/state_report.h"
#include "src/testing/testing.h"
#include "src/common/mathutil.h"
#include "src/middleware/fluid_levels.h"
#include "src/extras/topics_firmware/topics_firmware.h"
#include "src/drivers/cover_servo.h"

State s = CreateStateObject();

Controller controller;

int updatesInLastSecond;
unsigned long lastUpdatesPerSecondTime = millis();

// eepromStartup reads the startup counter, increments it, writes and prints it.
void eepromStartup() {
	uint8_t counter = I2C_EEPROM::ReadByte(STARTUP_COUNTER_MEM_ADDR);
	counter++;
	I2C_EEPROM::WriteByte(STARTUP_COUNTER_MEM_ADDR, counter);
	Logger::Info("Startup counter incremented to " + String(counter));
	s.startup_counter = counter;
}

void sleepHandler(Sleep::SleepStatus sleepStatus) {
	digitalWrite(STEPPER_SLEEP, LOW);

	RingLight::Off();

	if (Sleep::IsEStopActive()) {
		StateReport_SetStatus(machine_Status_E_STOP_ACTIVE);
		StateReport_Update(&s);
	}

	delay(1000);
}

void wakeHandler(Sleep::SleepStatus lastSleepStatus) {
	s.ClearState();

	RingLight::On();
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

	CoverServo_Init();

	initSteppers();

	// register callback
	SerialMQTT::SetTopicHandler(topicHandler);

	Sleep::SetOnWakeHandler(wakeHandler);
	Sleep::SetOnSleepHandler(sleepHandler);

	// disabling so it doesn't always start after flash / gateway restart
	// Sleep::Wake();

	eepromStartup();

	controller.Init(&s);

	Logger::Info("Sending first state report");
	StateReport_SetStatus(machine_Status_SLEEPING);
	StateReport_Update(&s);
	Logger::Info("setup complete");

	if (RUN_TESTS) {
		Logger::Info("Running tests on startup");
		while (1) {
			int status = Testing_RunTests();
			if (status == 0) {
				Logger::Info("Tests passed, continuing to main loop");
				break;
			}
			Logger::Error("Tests failed, retrying in 30s.");
			delay(30000);
		}
	}
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

void requestRinse() {
	if (s.rinseStatus == machine_RinseStatus_RINSE_COMPLETE) {
		s.rinseStatus = machine_RinseStatus_RINSE_REQUESTED;
		Logger::Info("requested rinse");
	} else {
		Logger::Error("Cannot request rinse because rinseStatus is not complete: " + String(s.rinseStatus));
	}
}

void topicHandler(String topic, String payload)
{
	Logger::Debug("topic handler start");
	if (topic == TOPIC_WAKE)
	{
		Sleep::Wake();
		return;
	}
	else if (topic == TOPIC_STATE_REPORT_REQUEST)
	{
		StateReport_ForceSend();
	}
	if (Sleep::IsSleeping())
	{
		// if asleep, only listen for wake and state report
		return;
	}

	if (topic == TOPIC_SLEEP)
	{
		Sleep::Sleep(Sleep::UNKNOWN);
	}
	else if (topic == TOPIC_SHUTDOWN)
	{
		s.shutdownRequested = true;
	}
	else if (topic == TOPIC_UNCALIBRATE)
	{
		s.pitchStepper.MarkAsNotCalibrated();
		s.yawStepper.MarkAsNotCalibrated();
		s.zStepper.MarkAsNotCalibrated();
		s.ringStepper.MarkAsNotCalibrated();
		s.pipetteStepper.MarkAsNotCalibrated();
		s.calibrationCleared = true;
	}
	else if (topic == TOPIC_SET_VALVE)
	{
		String values[] = {"", ""};
		SerialMQTT::UnpackCommaSeparatedValues(payload, values, 2);
		machine_SolenoidValve valve = (machine_SolenoidValve)values[0].toInt();
		bool open = values[1] == "true";
		uint8_t pin = 0;

		if (valve == machine_SolenoidValve_VALVE_DRAIN) {
			pin = DRAINAGE_VALVE_RELAY;
		} else if (valve == machine_SolenoidValve_VALVE_MILK) {
			pin = MILK_VALVE_RELAY;
		} else if (valve == machine_SolenoidValve_VALVE_WATER) {
			pin = WATER_VALVE_RELAY;
		} else if (valve == machine_SolenoidValve_VALVE_AIR) {
			pin = AIR_VALVE_RELAY;
		} else {
			Logger::Error("unknown valve " + String(valve));
			return;
		}
		SetDualRelay(pin, open);
		Logger::Info("set valve " + String(valve) + " to " + String(open));

		// to make this play nice as an override
		FluidLevels_WriteBowlLevel(0);
	}
	else if (topic == TOPIC_DISPENSE) {
		String values[] = {""};
		SerialMQTT::UnpackCommaSeparatedValues(payload, values, 1);
		float ul = values[0].toFloat();
		if (!s.pipetteState.spent) {
			s.pipetteState.dispenseRequested = true;
			s.pipetteState.ulVolumeHeldTarget -= ul;
			s.pipetteState.dispenseRequestNumber++;
			if (s.pipetteState.ulVolumeHeldTarget <= 0) {
				s.pipetteState.ulVolumeHeldTarget = 0;
			}
			Logger::Info("dispensed " + String(ul) + ", ulVolumeHeldTarget is now " + String(s.pipetteState.ulVolumeHeldTarget));
		} else {
			Logger::Info("Cannot dispense because already spent");
		}
	}
	else if (topic == TOPIC_COLLECT) {
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
	else if (topic == TOPIC_RINSE) {
		requestRinse();
	}
	else if (topic == TOPIC_GOTO_NODE)
	{
		long num = payload.toInt();
		s.forceIdleLocation = num == machine_Node_IDLE_LOCATION;
		s.SetGlobalNavigationTarget((machine_Node)num);
		Logger::Debug("Set globalTargetNode to " + String(num));
	}
	else if (topic == TOPIC_GOTO_XY) {
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
		boundToSignedMaximum(&yaw, MAX_BOWL_YAW);
		Logger::Info("Setting x,y, and target_ring=" + String(ring) + " and target_yaw=" + String(yaw));
		s.target_x = target_x;
		s.target_y = target_y;
		s.target_ring = ring;
		s.target_yaw = yaw;
	}
	else if (topic == TOPIC_TOGGLE_MANUAL)
	{
		s.manualRequested = !s.manualRequested;
		Logger::Info("Toggled manualRequested mode to " + String(s.manualRequested));
	}
	else if (topic == TOPIC_PIN_ON)
	{
		uint8_t pin = (uint8_t) payload.toInt();
		digitalWrite(pin, HIGH);
	}
	else if (topic == TOPIC_PIN_OFF)
	{
		uint8_t pin = (uint8_t) payload.toInt();
		digitalWrite(pin, LOW);
	}
	else if (topic == TOPIC_FLUID) {
		String values[] = {"", "", ""};
		SerialMQTT::UnpackCommaSeparatedValues(payload, values, 3);
		FluidType fluidType = (FluidType) values[0].toInt();
		float volume_ml = values[1].toFloat();
		bool open_drain = values[2] == "true";

		controller.NewFluidRequest(&s, fluidType, volume_ml, open_drain);
	}
	else if (topic == TOPIC_SET_IK_Z) {
		float z = payload.toFloat();
		if (z < MIN_BOWL_Z || z > s.zStepper.GetMaxUnit()) {
			Logger::Error("z level " + payload + " out of range.");
			return;
		}
		s.ik_target_z = z;
	}
	else if (topic == TOPIC_MARK_SAFE_TO_CALIBRATE) {
		s.overrideCalibrationBlock = true;
		Logger::Info("Set overrideCalibrationBlock true per mqtt request");
	}
	else if (topic == TOPIC_SET_COVER_SERVO_US) {
		int us = payload.toInt();
		CoverServo_SetMicroseconds(us);
	}
	else if (topic == TOPIC_SET_COVER_OPEN) {
		CoverServo_Open();
	}
	else if (topic == TOPIC_SET_COVER_CLOSE) {
		CoverServo_Close();
	}
	else if (topic == TOPIC_MAINTENANCE) {
		s.target_ring = MAINTENANCE_RING_ANGLE;
		s.forceIdleLocation = false;
		Navigation::SetGlobalNavigationTarget(&s, machine_Node_OUTER_HANDOVER);
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
	// SerialMQTT::Publish("mega/d/S_A", String(digitalRead(SWITCH_A)));
	// SerialMQTT::Publish("mega/d/S_B", String(digitalRead(SWITCH_B)));
	// SerialMQTT::Publish("mega/d/B_A", String(digitalRead(BUTTON_A)));

	// Power
	// SerialMQTT::Publish("mega/d/V12_C", String(analogRead(V12_CURRENT)));
	// SerialMQTT::Publish("mega/d/V5_C", String(analogRead(V5_CURRENT)));

	// RX Controller data
	// FS_I6::PrintRawChannels();
	// FS_I6::PrintProcessedChannels();

	// stepper raw position
	// SerialMQTT::Publish("mega/d/R_POS", String(s.ringStepper.currentPosition()));
	// SerialMQTT::Publish("mega/d/Z_POS", String(s.zStepper.currentPosition()));
	// SerialMQTT::Publish("mega/d/Y_POS", String(s.yawStepper.currentPosition()));
	// SerialMQTT::Publish("mega/d/P_POS", String(s.pitchStepper.currentPosition()));
	// SerialMQTT::Publish("mega/d/PP_POS", String(s.pipetteStepper.currentPosition()));

	// stepper units
	SerialMQTT::Publish("mega/d/R_UNIT", String(s.ringStepper.PositionToUnit(s.ringStepper.currentPosition())));
	// SerialMQTT::Publish("mega/d/Z_UNIT", String(s.zStepper.PositionToUnit(s.zStepper.currentPosition())));
	// SerialMQTT::Publish("mega/d/Y_UNIT", String(s.yawStepper.PositionToUnit(s.yawStepper.currentPosition())));
	// SerialMQTT::Publish("mega/d/P_UNIT", String(s.pitchStepper.PositionToUnit(s.pitchStepper.currentPosition())));
	// SerialMQTT::Publish("mega/d/PP_UNIT", String(s.pipetteStepper.PositionToUnit(s.pipetteStepper.currentPosition())));

	// SerialMQTT::Publish("mega/d/PP_L_SW", String(digitalRead(PIPETTE_LIMIT_SWITCH)));
	// SerialMQTT::Publish("mega/d/PP_CALI", String(s.pipetteStepper.IsCalibrated()));

	// SerialMQTT::Publish("mega/d/DATA_MS", String(millis() - start));
	// SerialMQTT::Publish("mega/d/UPS", String(s.updatesPerSecond));
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
		StateReport_Update(&s);
		delay(200);
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
