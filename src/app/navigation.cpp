#include "navigation.h"
#include "../app/state.h"
#include "../middleware/logger.h"
#include "../middleware/sleep.h"
#include "../calibration.h"

// atTarget checks if z, pitch, and yaw steppers are approximately at their local (next) target node
bool atLocalTargetNode(State *s) {
	bool result = s->zStepper.AtTarget() && s->pitchStepper.AtTarget() && s->yawStepper.AtTarget();
	return result;
}

Node calculateNextNode(Node lastNode, Node targetNode) {
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
		if (targetNode >= MIN_VIAL_ABOVE && targetNode <= MAX_VIAL_ABOVE && targetNode % 10 == 0)
			return targetNode;
		if (targetNode >= MIN_VIAL_INSIDE && targetNode <= MAX_VIAL_INSIDE && targetNode % 10 == 5)
			// go to to correct above vial node
			return (Node) (targetNode - 5);
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

	// Movement from positions directly above vials
	if (lastNode >= MIN_VIAL_ABOVE && lastNode <= MAX_VIAL_ABOVE && lastNode % 10 == 0)
	{
		if (targetNode <= HOME_TOP)
			return HOME_TOP;
		if (targetNode >= OUTER_HANDOVER)
			return LOW_ENTRY_POINT;
		if (targetNode >= MIN_VIAL_ABOVE && targetNode <= MAX_VIAL_ABOVE && targetNode % 10 == 0)
			return targetNode;
		// If going inside a vial
		if (targetNode >= MIN_VIAL_INSIDE && targetNode <= MAX_VIAL_INSIDE && targetNode % 10 == 5) {
			// If we're above the correct vial
			if (lastNode == (Node) (targetNode - 5))
				return targetNode;
			else
				// go to to correct above vial node
				return (Node) (targetNode - 5);
		}
	}

	if (lastNode >= MIN_VIAL_INSIDE && lastNode <= MAX_VIAL_INSIDE && lastNode % 10 == 5) {
		// Drop back to above position
		if (lastNode != targetNode)
			return (Node) (lastNode - 5);
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
		if (targetNode == INVERSE_KINEMATICS_POSITION)
			return INVERSE_KINEMATICS_POSITION;
	}

	if (lastNode == INVERSE_KINEMATICS_POSITION) {
		if (targetNode < INVERSE_KINEMATICS_POSITION)
			return INNER_HANDOVER;
	}

	return UNDEFINED;
}

// note this will go directly to the specified node, so ensure it is a safe move!
void goToNode(State *s, Node node) {
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
		s->zStepper.moveTo(s->zStepper.UnitToPosition(s->zStepper.GetMinUnit()));
		s->pitchStepper.moveTo(s->pitchStepper.UnitToPosition(0));
		s->yawStepper.moveTo(s->yawStepper.UnitToPosition(s->yawStepper.GetMinUnit()));
		return;
	}
	else if (node == HOME_TOP)
	{
		s->zStepper.moveTo(s->zStepper.UnitToPosition(HOME_TOP_Z));
		s->pitchStepper.moveTo(s->pitchStepper.UnitToPosition(0));
		s->yawStepper.moveTo(s->yawStepper.UnitToPosition(YAW_ZERO_OFFSET));
		return;
	}

	// vial nodes (above)
	if (node >= MIN_VIAL_ABOVE && node <= MAX_VIAL_ABOVE && node % 10 == 0)
	{
		s->zStepper.moveTo(s->zStepper.UnitToPosition(HOME_TOP_Z));
		s->pitchStepper.moveTo(s->pitchStepper.UnitToPosition(VIAL_PITCH));

		int index = (node - MIN_VIAL_ABOVE) / 10;
		float yaw = VIAL_YAW_OFFSET + index * VIAL_YAW_INCREMENT;
		s->yawStepper.moveTo(s->yawStepper.UnitToPosition(yaw));
		return;
	}

	if (node >= MIN_VIAL_INSIDE && node <= MAX_VIAL_INSIDE && node % 10 == 5)
	{
		// nowhere to go, the ABOVE node is inside the region of the inside node.
		// so we let the above position carry over. Movement within this node
		// is controlled from outside the navigation system, to reduce complexity
		// here. 
		return;
	}

	if (node == OUTER_HANDOVER)
	{
		s->zStepper.moveTo(s->zStepper.UnitToPosition(HANDOVER_Z));
		s->pitchStepper.moveTo(s->pitchStepper.UnitToPosition(HANDOVER_PITCH));
		s->yawStepper.moveTo(s->yawStepper.UnitToPosition(HANDOVER_OUTER_YAW));
		return;
	}

	if (node == INNER_HANDOVER)
	{
		s->zStepper.moveTo(s->zStepper.UnitToPosition(IK_Z));
		s->pitchStepper.moveTo(s->pitchStepper.UnitToPosition(HANDOVER_PITCH));
		s->yawStepper.moveTo(s->yawStepper.UnitToPosition(HANDOVER_INNER_YAW));
		return;
	}

	if (node == INVERSE_KINEMATICS_POSITION)
	{
		// nowhere to go, movement is handled outside the navigation system.
		return;
	}
}

// return true if currently navigating between nodes
bool atGlobalTarget(State *s)
{
	return s->lastNode == s->globalTargetNode;
}

void atGlobalNodeHandler(Node node) {
	//todo: maybe write the node to eeprom so state can be restored.
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
		return SUCCESS;
	}

	// Calculate next local target
	Node localTargetNode = calculateNextNode(s->lastNode, s->globalTargetNode);
	Logger::Debug("calculateNextNode: last node " + String(s->lastNode) + " -> (" + String(localTargetNode) + ") -> global target " + String(s->globalTargetNode));
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
			atGlobalNodeHandler(s->lastNode);
			return SUCCESS;
		}
	}

	return RUNNING;
}

void Navigation::SetGlobalNavigationTarget(State *s, Node n) {
	// nothing to do
	if (s->globalTargetNode == n) return;

	Logger::Debug("Changing global target " + String(s->globalTargetNode) + " -> " + String(n));
	s->globalTargetNode = n;

	if (s->lastNode == s->localTargetNode || s->localTargetNode == UNDEFINED) {
		// no need for fancy checks if we're at our latest target
		return;
	}

	Node newLocalTarget = calculateNextNode(s->lastNode, n);

	if (s->localTargetNode != newLocalTarget) {
		//! local target will change, we must revist lastNode to be safe.
		Node oldLastNode = s->lastNode;
		// This is a reversal of direction. So, we pretend we were coming from 
		// the old local target through this operation:
		s->lastNode = s->localTargetNode;

		Logger::Debug("... Local target changing " + String(s->localTargetNode) + " -> " + String(newLocalTarget));
		Logger::Debug("... Therefore changing last node " + String(oldLastNode) + " -> " + String(s->localTargetNode));

		s->globalTargetNode = n;
	}

}