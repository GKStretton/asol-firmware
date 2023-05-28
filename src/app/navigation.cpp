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

machine_Node calculateNextNode(machine_Node lastNode, machine_Node targetNode) {
	if (lastNode == machine_Node_UNDEFINED || targetNode == machine_Node_UNDEFINED)
		return machine_Node_UNDEFINED;

	if (lastNode == targetNode)
		return targetNode;

	if (lastNode == machine_Node_HOME)
	{
		if (targetNode > machine_Node_HOME)
		{
			return machine_Node_HOME_TOP;
		}
	}

	if (lastNode == machine_Node_HOME_TOP)
	{
		if (targetNode == machine_Node_HOME)
			return machine_Node_HOME;
		if (targetNode >= machine_Node_MIN_VIAL_ABOVE && targetNode <= machine_Node_MAX_VIAL_ABOVE && targetNode % 10 == 0)
			return targetNode;
		if (targetNode >= machine_Node_MIN_VIAL_INSIDE && targetNode <= machine_Node_MAX_VIAL_INSIDE && targetNode % 10 == 5)
			// go to to correct above vial node
			return (machine_Node) (targetNode - 5);
		if (targetNode >= machine_Node_RINSE_CONTAINER_ENTRY)
			return machine_Node_LOW_ENTRY_POINT;
	}

	if (lastNode == machine_Node_LOW_ENTRY_POINT)
	{
		if (targetNode <= machine_Node_HOME_TOP)
			return machine_Node_HOME_TOP;
		if (targetNode >= machine_Node_RINSE_CONTAINER_ENTRY && targetNode <= machine_Node_RINSE_CONTAINER_LOW)
			return machine_Node_RINSE_CONTAINER_ENTRY;
		if (targetNode >= machine_Node_OUTER_HANDOVER)
			return machine_Node_OUTER_HANDOVER;
	}

	// Movement from positions directly above vials
	if (lastNode >= machine_Node_MIN_VIAL_ABOVE && lastNode <= machine_Node_MAX_VIAL_ABOVE && lastNode % 10 == 0)
	{
		if (targetNode <= machine_Node_HOME_TOP)
			return machine_Node_HOME_TOP;
		if (targetNode >= machine_Node_RINSE_CONTAINER_ENTRY)
			return machine_Node_LOW_ENTRY_POINT;
		if (targetNode >= machine_Node_MIN_VIAL_ABOVE && targetNode <= machine_Node_MAX_VIAL_ABOVE && targetNode % 10 == 0)
			return targetNode;
		// If going inside a vial
		if (targetNode >= machine_Node_MIN_VIAL_INSIDE && targetNode <= machine_Node_MAX_VIAL_INSIDE && targetNode % 10 == 5) {
			// If we're above the correct vial
			if (lastNode == (machine_Node) (targetNode - 5))
				return targetNode;
			else
				// go to to correct above vial node
				return (machine_Node) (targetNode - 5);
		}
	}

	if (lastNode >= machine_Node_MIN_VIAL_INSIDE && lastNode <= machine_Node_MAX_VIAL_INSIDE && lastNode % 10 == 5) {
		// Drop back to above position
		if (lastNode != targetNode)
			return (machine_Node) (lastNode - 5);
	}

	if (lastNode == machine_Node_RINSE_CONTAINER_ENTRY) {
		if (targetNode < machine_Node_RINSE_CONTAINER_ENTRY)
			return machine_Node_LOW_ENTRY_POINT;
		if (targetNode >= machine_Node_OUTER_HANDOVER)
			return machine_Node_OUTER_HANDOVER;
		if (targetNode == machine_Node_RINSE_CONTAINER_LOW)
			return machine_Node_RINSE_CONTAINER_LOW;
	}

	if (lastNode == machine_Node_RINSE_CONTAINER_LOW) {
		if (targetNode != machine_Node_RINSE_CONTAINER_LOW)
			return machine_Node_RINSE_CONTAINER_ENTRY;
	}

	if (lastNode == machine_Node_OUTER_HANDOVER)
	{
		if (targetNode >= machine_Node_RINSE_CONTAINER_ENTRY && targetNode <= machine_Node_RINSE_CONTAINER_LOW)
			return machine_Node_RINSE_CONTAINER_ENTRY;
		if (targetNode < machine_Node_OUTER_HANDOVER)
			return machine_Node_LOW_ENTRY_POINT;
		if (targetNode >= machine_Node_INNER_HANDOVER)
			return machine_Node_INNER_HANDOVER;
	}

	if (lastNode == machine_Node_INNER_HANDOVER)
	{
		if (targetNode <= machine_Node_OUTER_HANDOVER)
			return machine_Node_OUTER_HANDOVER;
		if (targetNode == machine_Node_INVERSE_KINEMATICS_POSITION)
			return machine_Node_INVERSE_KINEMATICS_POSITION;
	}

	if (lastNode == machine_Node_INVERSE_KINEMATICS_POSITION) {
		if (targetNode < machine_Node_INVERSE_KINEMATICS_POSITION)
			return machine_Node_INNER_HANDOVER;
	}

	return machine_Node_UNDEFINED;
}

// note this will go directly to the specified node, so ensure it is a safe move!
void goToNode(State *s, machine_Node node) {
	// define positions for each node
	if (node == machine_Node_UNDEFINED)
	{
		s->zStepper.stop();
		s->pitchStepper.stop();
		s->yawStepper.stop();
		Logger::Error("goToNode UNDEFINED, stopping steppers");
		return;
	}
	else if (node == machine_Node_HOME)
	{
		s->zStepper.moveTo(s->zStepper.UnitToPosition(s->zStepper.GetMinUnit()));
		s->pitchStepper.moveTo(s->pitchStepper.UnitToPosition(0));
		s->yawStepper.moveTo(s->yawStepper.UnitToPosition(s->yawStepper.GetMinUnit()));
		return;
	}
	else if (node == machine_Node_HOME_TOP)
	{
		s->zStepper.moveTo(s->zStepper.UnitToPosition(HOME_TOP_Z));
		s->pitchStepper.moveTo(s->pitchStepper.UnitToPosition(0));
		s->yawStepper.moveTo(s->yawStepper.UnitToPosition(YAW_ZERO_OFFSET));
		return;
	}

	// vial nodes (above)
	if (node >= machine_Node_MIN_VIAL_ABOVE && node <= machine_Node_MAX_VIAL_ABOVE && node % 10 == 0)
	{
		s->zStepper.moveTo(s->zStepper.UnitToPosition(HOME_TOP_Z));
		s->pitchStepper.moveTo(s->pitchStepper.UnitToPosition(VIAL_PITCH));

		int index = (node - machine_Node_MIN_VIAL_ABOVE) / 10;
		float yaw = VIAL_YAW_OFFSET + index * VIAL_YAW_INCREMENT;
		s->yawStepper.moveTo(s->yawStepper.UnitToPosition(yaw));
		return;
	}

	if (node >= machine_Node_MIN_VIAL_INSIDE && node <= machine_Node_MAX_VIAL_INSIDE && node % 10 == 5)
	{
		// nowhere to go, the ABOVE node is inside the region of the inside node.
		// so we let the above position carry over. Movement within this node
		// is controlled from outside the navigation system, to reduce complexity
		// here. 
		return;
	}

	if (node == machine_Node_RINSE_CONTAINER_ENTRY)
	{
		s->zStepper.moveTo(s->zStepper.UnitToPosition(RINSE_CONTAINER_ENTRY_Z));
		s->pitchStepper.moveTo(s->pitchStepper.UnitToPosition(RINSE_CONTAINER_PITCH));
		s->yawStepper.moveTo(s->yawStepper.UnitToPosition(RINSE_CONTAINER_YAW));
		return;
	}

	if (node == machine_Node_RINSE_CONTAINER_LOW)
	{
		s->zStepper.moveTo(s->zStepper.UnitToPosition(RINSE_CONTAINER_LOW_Z));
		s->pitchStepper.moveTo(s->pitchStepper.UnitToPosition(RINSE_CONTAINER_PITCH));
		s->yawStepper.moveTo(s->yawStepper.UnitToPosition(RINSE_CONTAINER_YAW));
		return;
	}

	if (node == machine_Node_OUTER_HANDOVER)
	{
		s->zStepper.moveTo(s->zStepper.UnitToPosition(HANDOVER_Z));
		s->pitchStepper.moveTo(s->pitchStepper.UnitToPosition(HANDOVER_PITCH));
		s->yawStepper.moveTo(s->yawStepper.UnitToPosition(HANDOVER_OUTER_YAW));
		return;
	}

	if (node == machine_Node_INNER_HANDOVER)
	{
		s->zStepper.moveTo(s->zStepper.UnitToPosition(IK_Z));
		s->pitchStepper.moveTo(s->pitchStepper.UnitToPosition(HANDOVER_PITCH));
		s->yawStepper.moveTo(s->yawStepper.UnitToPosition(HANDOVER_INNER_YAW));
		return;
	}

	if (node == machine_Node_INVERSE_KINEMATICS_POSITION)
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

void atGlobalNodeHandler(machine_Node node) {
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
	if (s->localTargetNode != machine_Node_UNDEFINED && atGlobalTarget(s)) {
		return SUCCESS;
	}

	// Calculate next local target
	machine_Node localTargetNode = calculateNextNode(s->lastNode, s->globalTargetNode);
	Logger::Debug("calculateNextNode: last node " + String(s->lastNode) + " -> (" + String(localTargetNode) + ") -> global target " + String(s->globalTargetNode));
	if (localTargetNode == machine_Node_UNDEFINED) {
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

void Navigation::SetGlobalNavigationTarget(State *s, machine_Node n) {
	// nothing to do
	if (s->globalTargetNode == n) return;

	Logger::Debug("Changing global target " + String(s->globalTargetNode) + " -> " + String(n));
	s->globalTargetNode = n;

	if (s->lastNode == s->localTargetNode || s->localTargetNode == machine_Node_UNDEFINED) {
		// no need for fancy checks if we're at our latest target
		return;
	}

	machine_Node newLocalTarget = calculateNextNode(s->lastNode, n);

	if (s->localTargetNode != newLocalTarget) {
		//! local target will change, we must revist lastNode to be safe.
		machine_Node oldLastNode = s->lastNode;
		// This is a reversal of direction. So, we pretend we were coming from 
		// the old local target through this operation:
		s->lastNode = s->localTargetNode;

		Logger::Debug("... Local target changing " + String(s->localTargetNode) + " -> " + String(newLocalTarget));
		Logger::Debug("... Therefore changing last node " + String(oldLastNode) + " -> " + String(s->localTargetNode));

		s->globalTargetNode = n;
	}

}