#include "ik_algorithm.h"
#include "../middleware/logger.h"
#include "../common/mathutil.h"
#include "../calibration.h"

int getRingAndYawFromXY(float x, float y, float lastRing, float *ring, float *yaw, float minRingUnit, float maxRingUnit)
{
	boundXYToCircle(&x, &y, IK_TARGET_RADIUS_FRAC);
	// No solutions at centre so just do ring = last, yawOffset = 0
	if (abs(x) <= 0.001 && abs(y) <= 0.001)
	{
		*yaw = 0;
		*ring = lastRing;
		Logger::Debug("Centre point, so yawOffset = 0 and ring = " + String(lastRing) + " (last value)");
		return 0;
	}

	Logger::Debug("Target: " + String(x) + ", " + String(y));

	float x_mm = x * STAGE_RADIUS_MM;
	float y_mm = y * STAGE_RADIUS_MM;

	double xi, yi, xi_prime, yi_prime;

	// Find the intersections of two circles about the centre and target
	// respectively, with radii equal to the arm length.
	int case_ = CircleCircleIntersection(
		0, 0, ARM_PATH_RADIUS_MM,
		x_mm, y_mm, ARM_PATH_RADIUS_MM,
		&xi, &yi, &xi_prime, &yi_prime);

	if (case_ == 0)
	{
		Logger::Warn("No solutions to circle intersection! Invalid calibration?");
		return 1;
	}

	// Now we have the 2 intersect points, and target x_mm, y_mm.
	// Calculate the two outer ring angles from the intersection points
	double angle = fmod(-atan2(xi, yi) * 180.0 / M_PI + 270.0, 360.0);
	double angle_prime = fmod(-atan2(xi_prime, yi_prime) * 180.0 / M_PI + 270.0, 360.0);

	Logger::Debug("angle=" + String(angle) + " angle_prime=" + String(angle_prime));

	// Now we work out which solution to use
	bool use_i_prime = false;
	// if they're both valid, choose the one that requires least movement
	if (numInRange(angle, minRingUnit, maxRingUnit) && numInRange(angle_prime, minRingUnit, maxRingUnit))
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
	// otherwise, select the only valid one
	else if (numInRange(angle, minRingUnit, maxRingUnit))
	{
		*ring = (float)angle;
	}
	// otherwise, select the only valid one
	else if (numInRange(angle_prime, minRingUnit, maxRingUnit))
	{
		use_i_prime = true;
		*ring = (float)angle_prime;
	}
	else
	{
		// *ring = (float)5.0;
		Logger::Warn("Both intersection angles (" +
					 String(angle) + ", " + String(angle_prime) +
					 ") are out of ring angle bounds. Aborting");
		return 1;
	};

	Logger::Debug(use_i_prime ? "Chose angle_prime" : "Chose angle");

	// Calculate arm yaw. It's the angle between the vectors from the robot to
	// a) the centre of the bowl and b) the target.
	double newYaw;
	if (use_i_prime)
	{
		newYaw = -AngleBetweenVectors(-xi_prime, -yi_prime, x_mm - xi_prime, y_mm - yi_prime);
	}
	else
	{
		newYaw = -AngleBetweenVectors(-xi, -yi, x_mm - xi, y_mm - yi);
	}

	// result is set by pointer
	*yaw = (float)newYaw;
	Logger::Debug("Set ring=" + String(*ring) + " and newYaw=" + String(newYaw));

	// return no error
	return 0;
}