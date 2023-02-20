#pragma once

// Basic Stepper calibration

// After a limit switch is pressed, the current unit is set to the min unit minus
// this value. So the motor would have to move at least this many units below
// minimum to re-trigger the limit switch
#define CALIBRATION_GAP_SIZE 1

// multiplier for stepper speeds and accelerations. Useful for cautious testing.
#define SPEED_MULT 1

// ul of air to draw in from 0 position before taking in liquid
#define PIPETTE_BUFFER 200
// z level from which to take in liquid. (may become dynamic based on dye level in future.)
#define PIPETTE_INTAKE_Z 40

// Physical calibration of arm positions etc
#define STAGE_RADIUS_MM 75
#define ARM_PATH_RADIUS_MM 164.7

#define YAW_ZERO_OFFSET -21.3
#define RING_ZERO_OFFSET 24.8

#define CENTRE_PITCH 48.95
#define MIN_BOWL_Z 32.5

// NODE CALIBRATION (UNITS)
#define HOME_TOP_Z 73

#define VIAL_PITCH 0
#define VIAL_YAW_OFFSET -6
#define VIAL_YAW_INCREMENT 36.38

#define IK_Z 42
#define HANDOVER_Z 50
#define HANDOVER_PITCH 70
#define HANDOVER_INNER_YAW 15
#define HANDOVER_OUTER_YAW 50

// the maximum value of yaw that it's safe to enter ik mode within
#define MAX_BOWL_YAW 24

// number of milliseconds to open the air valve for after fluid dispense
#define FLUID_TRAVEL_TIME_MS 5000
#define WATER_VOLUME_PER_SECOND_ML 5.5
#define MILK_VOLUME_PER_SECOND_ML 6.0
#define DRAIN_VOLUME_PER_SECOND_ML 4.0
#define MAX_FLUID_LEVEL 500.0