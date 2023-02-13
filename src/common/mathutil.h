#pragma once

/* CircleCircleIntersection() *
 * Determine the points where 2 circles in a common plane intersect.
 *
 * int CircleCircleIntersection(
 *                                // center and radius of 1st circle
 *                                double x0, double y0, double r0,
 *                                // center and radius of 2nd circle
 *                                double x1, double y1, double r1,
 *                                // 1st intersection point
 *                                double *xi, double *yi,              
 *                                // 2nd intersection point
 *                                double *xi_prime, double *yi_prime)
 *
 * This is a public domain work. 3/26/2005 Tim Voght
 *
 */
int CircleCircleIntersection(double x0, double y0, double r0,
							   double x1, double y1, double r1,
							   double *xi, double *yi,
							   double *xi_prime, double *yi_prime);

// AngleBetweenVectors returns angle in degrees between two specified vectors
double AngleBetweenVectors(double x0, double y0, double x1, double y1);

void boundXYToCircle(float *x, float *y, float radius);
void boundToSignedMaximum(float *n, float range);

bool numInRange(float num, float min, float max);