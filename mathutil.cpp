#include "mathutil.h"
#include <math.h>

double AngleBetweenVectors(double x0, double y0, double x1, double y1) {
    double dot = x0*x1 + y0*y1;
    double det = x0*y1 - y0*x1;
    return atan2(det, dot) * 180 / M_PI;
}