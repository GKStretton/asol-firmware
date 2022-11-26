#pragma once

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