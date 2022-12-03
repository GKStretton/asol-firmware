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

	// Above and inside test tube positions
	// Note, INSIDE positions are valid for a range of z values, determined outside Navigation.
	VIAL_1_ABOVE = 10,
	MIN_VIAL_ABOVE = VIAL_1_ABOVE,
	VIAL_1_INSIDE = 15,
	MIN_VIAL_INSIDE = VIAL_1_INSIDE,

	VIAL_2_ABOVE = 20,
	VIAL_2_INSIDE = 25,

	VIAL_3_ABOVE = 30,
	VIAL_3_INSIDE = 35,

	VIAL_4_ABOVE = 40,
	VIAL_4_INSIDE = 45,

	VIAL_5_ABOVE = 50,
	VIAL_5_INSIDE = 55,

	VIAL_6_ABOVE = 60,
	VIAL_6_INSIDE = 65,

	VIAL_7_ABOVE = 70,
	MAX_VIAL_ABOVE = VIAL_7_ABOVE,
	VIAL_7_INSIDE = 75,
	MAX_VIAL_INSIDE = VIAL_7_INSIDE,

	// The node to enter the lower (vial) regions at
	LOW_ENTRY_POINT = VIAL_3_ABOVE,

	OUTER_HANDOVER = 90,
	INNER_HANDOVER = 110,
	INVERSE_KINEMATICS_POSITION = 150,

	IDLE_LOCATION = HOME_TOP
};

// indexed by 1, convert number 1-n to Node.
Node VialNumberToInsideNode(int number);