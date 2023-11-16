#include "mathUtils.h"

double map(double x, double a, double b, double c, double d) {
    return (x - a) / (b - a) * (d - c) + c;
}

double dist(double x1, double y1, double x2, double y2) {
    return sqrt((x2- x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}