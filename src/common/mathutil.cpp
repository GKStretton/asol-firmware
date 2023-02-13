#include "mathutil.h"
#include <math.h>

double AngleBetweenVectors(double x0, double y0, double x1, double y1) {
	double dot = x0*x1 + y0*y1;
	double det = x0*y1 - y0*x1;
	return atan2(det, dot) * 180 / M_PI;
}

void boundXYToCircle(float *x, float *y, float radius) {
	float mag = (float)hypotf((float)*x, (float)*y);
	if (mag > radius)
	{
		*x = *x / mag * radius;
		*y = *y / mag * radius;
	}
}

void boundToSignedMaximum(float *n, float range) {
	if (*n > range) {
		*n = range;
	}
	if (*n < -range) {
		*n = -range;
	}
}

bool numInRange(float num, float min, float max) {
	return num >= min && num <= max;
}