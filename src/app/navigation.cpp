#include "navigation.h"
#include "../app/state.h"
#include "../middleware/logger.h"
#include "../calibration.h"

// atTarget checks if z, pitch, and yaw steppers are approximately at their local (next) target node
bool atLocalTargetNode(State *s) {
	bool result = s->zStepper.AtTarget() && s->pitchStepper.AtTarget() && s->yawStepper.AtTarget();
	Logger::Debug("atLocalTargetNode");
	Logger::Debug("atLocalTargetNode returning " + String(result));
	Logger::Debug("zStepper current " + String(s->zStepper.currentPosition()));
	Logger::Debug("zStepper target " + String(s->zStepper.targetPosition()));
	Logger::Debug("pitchStepper current " + String(s->pitchStepper.currentPosition()));
	Logger::Debug("pitchStepper target " + String(s->pitchStepper.targetPosition()));
	Logger::Debug("yawStepper current " + String(s->yawStepper.currentPosition()));
	Logger::Debug("yawStepper target " + String(s->yawStepper.targetPosition()));
	return result;
}

Node calculateNextNode(Node lastNode, Node targetNode) {
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
		if (targetNode >= MIN_VIAL && targetNode <= MAX_VIAL)
			return targetNode;
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
		s->yawStepper.moveTo(s->yawStepper.UnitToPosition(YAW_ZERO_OFFSET));
		return;
	}
	else if (node == HOME_TOP)
	{
		s->zStepper.moveTo(s->zStepper.UnitToPosition(HOME_TOP_Z));
		s->pitchStepper.moveTo(s->pitchStepper.UnitToPosition(0));
		s->yawStepper.moveTo(s->yawStepper.UnitToPosition(YAW_ZERO_OFFSET));
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
bool atGlobalTarget(State *s)
{
	return s->lastNode == s->globalTargetNode;
}

// the update tick for node navigation
Status Navigation::UpdateNodeNavigation(State *s)
{
	// Prevent action if uncalibrated
	if (!s->IsArmCalibrated()) {
		Logger::Error("Cannot updateNodeNavigation because steppers aren't calibrated");
		return FAILURE;
	}

	// Don't take action if we're at the global target. But do if localtarget undefined (start)
	if (s->localTargetNode != UNDEFINED && atGlobalTarget(s)) {
		Logger::Debug("atGlobalTarget, so not updating node navigation");
		return SUCCESS;
	}

	// Calculate next local target
	Node localTargetNode = calculateNextNode(s->lastNode, s->globalTargetNode);
	Logger::Debug("calculateNextNode returned " + String(localTargetNode) + " for last node " + String(s->lastNode) + " and global target " + String(s->globalTargetNode));
	if (localTargetNode == UNDEFINED) {
		Logger::Debug("local target undefined, skipping node navigation");
		return FAILURE;
	}

	// Check for a change in local target, and set stepper positions if so
	if (s->localTargetNode != localTargetNode)
	{
		s->localTargetNode = localTargetNode;
		Logger::Debug("Local target changed to " + String(localTargetNode) + ". Setting stepper positions accordingly.");
		goToNode(s, localTargetNode);
	}

	// Check for arrival at local target
	if (atLocalTargetNode(s))
	{
		s->lastNode = localTargetNode;
		Logger::Debug("Arrived at local target. lastNode set to " + String(s->lastNode));
		if (atGlobalTarget(s)) {
			return SUCCESS;
		}
	}

	return RUNNING;
}