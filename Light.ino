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

//################# NODE STUFF ###########################
/*
Note, currently only the z, pitch, and yaw axes are managed by the node system.
This is because the ring and pipette axes have no effect on collision avoidance.
Furthermore it decouples those so that they can move independently / concurrently
to node navigation. For example, go to next target ring position while collecting
dye
*/
enum Node
{
	UNDEFINED = 0,
	HOME = 4,
	HOME_TOP = 8,
	// Above test tube positions
	VIAL_1 = 11,
	MIN_VIAL = VIAL_1,
	VIAL_2 = 12,
	VIAL_3 = 13,
	VIAL_4 = 14,
	VIAL_5 = 15,
	VIAL_6 = 16,
	VIAL_7 = 17,
	MAX_VIAL = VIAL_7,

	// The node to enter the lower (vial) regions at
	LOW_ENTRY_POINT = VIAL_3,

	OUTER_HANDOVER = 45,
	INNER_HANDOVER = 55,
};

struct State
{
	// The most recent node to have been visited
	Node lastNode;
	// Most recent local node being navigated to
	Node localTargetNode;
	// The final goal node in a potentially multi-hop movement
	Node globalTargetNode;
	// If true, respect the fs-i6 controller
	bool manual;
	// Timing
	unsigned long lastControlUpdate;
	unsigned long lastDataUpdate;

	// Steppers
	UnitStepper pitchStepper;
	UnitStepper yawStepper;
	UnitStepper zStepper;
	UnitStepper ringStepper;
	UnitStepper pipetteStepper;
} s = {
	HOME,
	UNDEFINED,
	UNDEFINED,
	false,
	0,
	0,
	UnitStepper(PITCH_STEPPER_STEP, PITCH_STEPPER_DIR, 16, 0.44, 0, 90),
	UnitStepper(YAW_STEPPER_STEP, YAW_STEPPER_DIR, 8, 0.36, YAW_ZERO_OFFSET, 198),
	UnitStepper(Z_STEPPER_STEP, Z_STEPPER_DIR, 4, 0.04078, 0, 73),
	UnitStepper(RING_STEPPER_STEP, RING_STEPPER_DIR, 32, 0.4, RING_ZERO_OFFSET, 280),
	UnitStepper(PIPETTE_STEPPER_STEP, PIPETTE_STEPPER_DIR, 32, 2.74, 100, 700),
};

// atTarget checks if z, pitch, and yaw steppers are approximately at their local (next) target node
bool atLocalTargetNode(State *s)
{
	bool result = s->zStepper.AtTarget() && s->pitchStepper.AtTarget() && s->yawStepper.AtTarget();
	Logger::Debug("atLocalTargetNode");
	Logger::Debug("atLocalTargetNode returning " + String(result));
	Logger::Debug("zStepper current " + String(s->zStepper.currentPosition()));
	Logger::Debug("zStepper target " + String(s->zStepper.targetPosition()));
	return result;
}

Node calculateNextNode(Node lastNode, Node targetNode)
{
	Logger::Debug("calculateNextNode for last " + String(lastNode) + " and target " + String(targetNode));
	if (lastNode == UNDEFINED || targetNode == UNDEFINED)
		return UNDEFINED;

	if (lastNode == targetNode)
		return targetNode;

	if (lastNode == HOME)
	{
		if (targetNode > HOME)
		{
			return HOME_TOP;
		}
	}

	if (lastNode == HOME_TOP)
	{
		if (targetNode == HOME)
			return HOME;
		if (targetNode >= MIN_VIAL && targetNode <= MAX_VIAL)
			return targetNode;
		if (targetNode >= OUTER_HANDOVER)
			return LOW_ENTRY_POINT;
	}

	if (lastNode == LOW_ENTRY_POINT)
	{
		if (targetNode <= HOME_TOP)
			return HOME_TOP;
		if (targetNode >= OUTER_HANDOVER)
			return OUTER_HANDOVER;
	}

	if (lastNode >= MIN_VIAL && lastNode <= MAX_VIAL)
	{
		if (targetNode <= HOME_TOP)
			return HOME_TOP;
		if (targetNode >= OUTER_HANDOVER)
			return LOW_ENTRY_POINT;
	}

	if (lastNode == OUTER_HANDOVER)
	{
		if (targetNode < OUTER_HANDOVER)
			return LOW_ENTRY_POINT;
		if (targetNode >= INNER_HANDOVER)
			return INNER_HANDOVER;
	}

	if (lastNode == INNER_HANDOVER)
	{
		if (targetNode <= OUTER_HANDOVER)
			return OUTER_HANDOVER;
	}

	return UNDEFINED;
}

// note this will go directly to the specified node, so ensure it is a safe move!
void goToNode(State *s, Node node)
{
	// define positions for each node
	if (node == UNDEFINED)
	{
		s->zStepper.stop();
		s->pitchStepper.stop();
		s->yawStepper.stop();
		Logger::Error("goToNode UNDEFINED, stopping steppers");
		return;
	}
	else if (node == HOME)
	{
		s->zStepper.moveTo(s->zStepper.UnitToPosition(0));
		s->pitchStepper.moveTo(s->pitchStepper.UnitToPosition(0));
		s->yawStepper.moveTo(s->yawStepper.UnitToPosition(0));
		return;
	}
	else if (node == HOME_TOP)
	{
		s->zStepper.moveTo(s->zStepper.UnitToPosition(HOME_TOP_Z));
		s->pitchStepper.moveTo(s->pitchStepper.UnitToPosition(0));
		s->yawStepper.moveTo(s->yawStepper.UnitToPosition(0));
		return;
	}

	// vial nodes
	if (node >= MIN_VIAL && node <= MAX_VIAL)
	{
		s->zStepper.moveTo(s->zStepper.UnitToPosition(HOME_TOP_Z));
		s->pitchStepper.moveTo(s->pitchStepper.UnitToPosition(VIAL_PITCH));

		int index = node - MIN_VIAL;
		float yaw = VIAL_YAW_OFFSET + index * VIAL_YAW_INCREMENT;
		s->yawStepper.moveTo(s->yawStepper.UnitToPosition(yaw));
		return;
	}

	if (node == OUTER_HANDOVER || node == INNER_HANDOVER)
	{
		s->zStepper.moveTo(s->zStepper.UnitToPosition(HANDOVER_Z));
		s->pitchStepper.moveTo(s->pitchStepper.UnitToPosition(HANDOVER_PITCH));

		float yaw = node == OUTER_HANDOVER ? HANDOVER_OUTER_YAW : HANDOVER_INNER_YAW;
		s->yawStepper.moveTo(s->yawStepper.UnitToPosition(yaw));
		return;
	}
}

// return true if currently navigating between nodes
bool navigatingNodes(State *s)
{
	return s->lastNode != s->globalTargetNode;
}

// return true if z, pitch, and yaw steppers are calibrated
bool calibrated(State *s)
{
	return s->zStepper.IsCalibrated() && s->pitchStepper.IsCalibrated() && s->yawStepper.IsCalibrated();
}

// the update tick for node navigation
void updateNodeNavigation(State *s)
{
	if (!navigatingNodes(s))
	{
		return;
	}
	if (!calibrated(s))
	{
		Logger::Error("Cannot updateNodeNavigation because steppers aren't calibrated");
		return;
	}

	Node localTargetNode = calculateNextNode(s->lastNode, s->globalTargetNode);
	Logger::Debug("calculateNextNode returned " + String(localTargetNode));
	if (localTargetNode == UNDEFINED)
	{
		return;
	}
	if (s->localTargetNode != localTargetNode)
	{
		s->localTargetNode = localTargetNode;
		Logger::Debug("Setting local target to " + String(localTargetNode));
		goToNode(s, localTargetNode);
	}

	if (atLocalTargetNode(s))
	{
		s->lastNode = localTargetNode;
		Logger::Debug("lastNode set to " + String(s->lastNode));
	}
}

// new update function for control over mqtt essentially
void autoUpdate(State *s)
{
	updateNodeNavigation(s);
}

//################# END NODE STUFF #######################

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

	s.pitchStepper.setMaxSpeed(1250 * SPEED_MULT);
	s.pitchStepper.setAcceleration(1600 * SPEED_MULT);
	s.pitchStepper.setPinsInverted(true);

	s.yawStepper.setMaxSpeed(1250 * SPEED_MULT);
	s.yawStepper.setAcceleration(1600 * SPEED_MULT);
	s.yawStepper.setPinsInverted(true);

	s.zStepper.setMaxSpeed(1250 * SPEED_MULT);
	s.zStepper.setAcceleration(800 * SPEED_MULT);

	s.ringStepper.setPinsInverted(true);
	s.ringStepper.setMaxSpeed(1250 * SPEED_MULT);
	s.ringStepper.setAcceleration(800 * SPEED_MULT);

	s.pipetteStepper.setMaxSpeed(1250 * SPEED_MULT);
	s.pipetteStepper.setAcceleration(800 * SPEED_MULT);
	s.pipetteStepper.setPinsInverted(true);

	s.ringStepper.SetLimitSwitchPin(RING_LIMIT_SWITCH);
	s.zStepper.SetLimitSwitchPin(Z_LIMIT_SWITCH);
	s.yawStepper.SetLimitSwitchPin(YAW_LIMIT_SWITCH);
	s.pitchStepper.SetLimitSwitchPin(PITCH_LIMIT_SWITCH);
	s.pipetteStepper.SetLimitSwitchPin(PIPETTE_LIMIT_SWITCH);


	// register callback
	SerialMQTT::SetTopicHandler(topicHandler);
	Logger::Info("setup complete");

	Sleep::Wake();
}

void topicHandler(String topic, String payload)
{
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
	else if (topic == "mega/req/goto-node")
	{
		long num = payload.toInt();
		s.globalTargetNode = (Node)num;
		Logger::Debug("Set globalTargetNode to " + String(num));
		// } else if (topic == "mega/req/goto-xy") {
		// todo: unpack payload into ints
		// Logger::Debug("goto-xy ACK:" + String(x_target) + String(y_target))
		// } else if (topic == "mega/req/dispense") {
		// } else if (topic == "mega/req/collect") {
	}
	else
	{
		Logger::Debug("no handler for " + topic + " (payload = " + payload + ")");
	}
}

void dataUpdate()
{
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
	SerialMQTT::Publish("d/R_POS", String(s.ringStepper.currentPosition()));
	SerialMQTT::Publish("d/Z_POS", String(s.zStepper.currentPosition()));
	SerialMQTT::Publish("d/Y_POS", String(s.yawStepper.currentPosition()));
	SerialMQTT::Publish("d/P_POS", String(s.pitchStepper.currentPosition()));
	SerialMQTT::Publish("d/PP_POS", String(s.pipetteStepper.currentPosition()));

	// stepper units
	SerialMQTT::Publish("d/R_UNIT", String(s.ringStepper.PositionToUnit(s.ringStepper.currentPosition())));
	SerialMQTT::Publish("d/Z_UNIT", String(s.zStepper.PositionToUnit(s.zStepper.currentPosition())));
	SerialMQTT::Publish("d/Y_UNIT", String(s.yawStepper.PositionToUnit(s.yawStepper.currentPosition())));
	SerialMQTT::Publish("d/P_UNIT", String(s.pitchStepper.PositionToUnit(s.pitchStepper.currentPosition())));
	SerialMQTT::Publish("d/PP_UNIT", String(s.pipetteStepper.PositionToUnit(s.pipetteStepper.currentPosition())));
}

//################# IK NAVIGATION / OLD CONTROLLER STUFF #######################

bool ringAngleValid(UnitStepper *ringStepper, double a)
{
	return a >= ringStepper->GetMinUnit() and a <= ringStepper->GetMaxUnit();
}

bool goToIKPosition()
{
	// todo: implement non-blocking goto ik logic
	return false;
}

bool goToPipettePosition()
{
	// todo: implement non-blocking goto pipette logic
	return false;
}

void boundXYToCircle(float *x, float *y, float radius)
{
	float mag = (float)hypotf((float)*x, (float)*y);
	if (mag > radius)
	{
		*x = *x / mag * radius;
		*y = *y / mag * radius;
	}
}

void updateRingAndYaw(float x, float y, float lastRing, float *ring, float *yaw, UnitStepper *ringStepper)
{
	boundXYToCircle(&x, &y, 0.7);
	// No solutions at centre so just do ring = last, yawOffset = 0
	if (abs(x) <= 0.001 && abs(y) <= 0.001)
	{
		*yaw = 0;
		*ring = lastRing;
		Logger::Debug("Centre point, so yawOffset = 0 and ring = " + String(lastRing) + " (last value)");
		return;
	}

	Logger::Debug("Target: " + String(x) + ", " + String(y));

	float x_mm = x * STAGE_RADIUS_MM;
	float y_mm = y * STAGE_RADIUS_MM;

	double xi, yi, xi_prime, yi_prime;

	int case_ = CircleCircleIntersection(
		0, 0, ARM_PATH_RADIUS_MM,
		x_mm, y_mm, ARM_PATH_RADIUS_MM,
		&xi, &yi, &xi_prime, &yi_prime);

	if (case_ == 0)
	{
		Logger::Warn("No solutions to circle intersection! Invalid calibration?");
		return;
	}

	// Now we have the 2 intersect points, and target x_mm, y_mm

	double angle = fmod(-atan2(xi, yi) * 180.0 / M_PI + 270.0, 360.0);
	double angle_prime = fmod(-atan2(xi_prime, yi_prime) * 180.0 / M_PI + 270.0, 360.0);

	Logger::Debug("angle=" + String(angle) + " angle_prime=" + String(angle_prime));

	bool use_i_prime = false;
	if (ringAngleValid(ringStepper, angle) && ringAngleValid(ringStepper, angle_prime))
	{
		// move to whichever is closest to previous position
		if (abs(angle - lastRing) < abs(angle_prime - lastRing))
		{
			*ring = (float)angle;
		}
		else
		{
			use_i_prime = true;
			*ring = (float)angle_prime;
		}
	}
	else if (ringAngleValid(ringStepper, angle))
	{
		*ring = (float)angle;
	}
	else if (ringAngleValid(ringStepper, angle_prime))
	{
		use_i_prime = true;
		*ring = (float)angle_prime;
	}
	else
	{
		*ring = (float)5.0;
		Logger::Warn("Both intersection angles (" +
					 String(angle) + ", " + String(angle_prime) +
					 ") are out of ring angle bounds. Setting to 5deg");
	};

	Logger::Debug(use_i_prime ? "Chose angle_prime" : "Chose angle");

	double newYaw;
	if (use_i_prime)
	{
		newYaw = -AngleBetweenVectors(-xi_prime, -yi_prime, x_mm - xi_prime, y_mm - yi_prime);
	}
	else
	{
		newYaw = -AngleBetweenVectors(-xi, -yi, x_mm - xi, y_mm - yi);
	}
	*yaw = (float)newYaw;
	Logger::Debug("Set ring=" + String(*ring) + " and newYaw=" + String(newYaw));
}

float x_target = 0;
float y_target = 0;

// ikModeUpdate does ik logic on an x and y in range (-1,1).
void ikModeUpdate(State *s)
{
	//! write this
	// bool inPosition = goToIKPosition();
	// if (!inPosition) {
	// 	return;
	// }

	if (FS_I6::GetSwitch(FS_I6::S2) == 2)
	{
		float dx = FS_I6::GetStick(FS_I6::RH);
		float dy = FS_I6::GetStick(FS_I6::RV);

		float sf = 0.05;
		x_target += sf * dx;
		y_target += sf * dy;
	}

	float ring, yaw;
	updateRingAndYaw(x_target, y_target,
					 s->ringStepper.PositionToUnit(s->ringStepper.currentPosition()),
					 &ring, &yaw, &(s->ringStepper));

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

// pippetteModeUpdate does pipette logic with
void pipetteModeUpdate()
{
	bool inPosition = goToPipettePosition();
	if (!inPosition)
	{
		return;
	}

	// todo pipette logic
}

void manualUpdate(State *s)
{
	// Get inputs

	int sw1 = FS_I6::GetSwitch(FS_I6::S1);
	int sw2 = FS_I6::GetSwitch(FS_I6::S2);

	bool boardSwitchA = digitalRead(SWITCH_A);

	// Sleep

	if (sw1)
	{
		Sleep::Wake();
	}

	digitalWrite(STEPPER_SLEEP, sw1 ? HIGH : LOW);

	// Nothing to do if steppers asleep
	if (!sw1)
		return;

	// Main control
	// bool assistedMode = boardSwitchA;

	float speedMult = 1600.0;

	if (sw2 == 0 || sw2 == 1)
	{
		// manual + pipette

		float left_h = FS_I6::GetStick(FS_I6::LH);
		if (sw2 == 0)
		{
			s->ringStepper.setSpeed(speedMult * left_h);
		}
		else if (sw2 == 1)
		{
			s->pipetteStepper.setSpeed(-speedMult * left_h);
		}
		float right_h = FS_I6::GetStick(FS_I6::RH);
		s->yawStepper.setSpeed(speedMult * right_h);
		float right_v = FS_I6::GetStick(FS_I6::RV);
		s->pitchStepper.setSpeed(speedMult * right_v);
		s->zStepper.SetMinUnit(0);
	}
	else if (sw2 == 2)
	{
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

// ##################### STEPPER META FUNCTIONALITY ####################

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

// ################ LOOP ###################

int updatesInLastSecond;
unsigned long lastUpdatesPerSecondTime = millis();
int updatesPerSecond;

void loop()
{
	SerialMQTT::Update();

	Sleep::Update();
	if (Sleep::IsSleeping())
	{
		return;
	}

	//? Maybe cache input

	// control
	if (millis() - s.lastControlUpdate > 100)
	{
		int sw1 = FS_I6::GetSwitch(FS_I6::S1);
		s.manual = sw1 != 0;

		if (s.manual)
		{
			manualUpdate(&s);
		}
		else
		{
			autoUpdate(&s);
		}
		s.lastControlUpdate = millis();
	}

	// actuation
	runSteppers(&s);

	if (PRINT_DATA && millis() - s.lastDataUpdate > 1000)
	{
		unsigned long now = millis();
		dataUpdate();
		SerialMQTT::Publish("d/DATA_MS", String(millis() - now));
		SerialMQTT::Publish("d/UPS", String(updatesPerSecond));
		Serial.println();
		s.lastDataUpdate = millis();
	}

	updatesInLastSecond++;
	if (millis() - lastUpdatesPerSecondTime > 1000)
	{
		updatesPerSecond = updatesInLastSecond;
		updatesInLastSecond = 0;
		lastUpdatesPerSecondTime = millis();
	}
}
