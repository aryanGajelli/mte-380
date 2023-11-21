#include "mathUtils.h"

#include "arm_math.h"
#include "debug.h"

double sign(double x) {
    return x < 0 ? -1 : 1;
}

double map(double x, double a, double b, double c, double d) {
    return (x - a) / (b - a) * (d - c) + c;
}

double dist(double x1, double y1, double x2, double y2) {
    return sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

double clamp(double x, double a, double b) {
    return x < a ? a : (x > b ? b : x);
}

double adjustTurn(double angle) {
    if (angle > 180)
        return angle - 360;
    else if (angle < -180)
        return angle + 360;
    return angle;
}

IntersectionType_E circleSegmentIntersection(vector3_t center, double radius, vector3_t p1, vector3_t p2, vector3_t *out1, vector3_t *out2) {
    // default no intersection
    out1->x = out2->x = NAN;
    out1->y = out2->y = NAN;

    // translate everything so that the center of the circle is at the origin
    p1.x -= center.x;
    p1.y -= center.y;
    p2.x -= center.x;
    p2.y -= center.y;

    double dx = p2.x - p1.x;
    double dy = p2.y - p1.y;
    double dr = sqrt(dx * dx + dy * dy);
    double dr_sq = dr * dr;
    double D = p1.x * p2.y - p2.x * p1.y;
    double discriminant = radius * radius * dr_sq - D * D;

    // no intersection
    if (discriminant < 0) {
        return NO_INTERSECTION;
    }

    double sqrt_discriminant = sqrt(discriminant);
    vector3_t sol1 = {.x = (D * dy + sign(dy) * dx * sqrt_discriminant) / dr_sq,
                      .y = (-D * dx + fabs(dy) * sqrt_discriminant) / dr_sq};

    vector3_t sol2 = {.x = (D * dy - sign(dy) * dx * sqrt_discriminant) / dr_sq,
                      .y = (-D * dx - fabs(dy) * sqrt_discriminant) / dr_sq};

    vector3_t min = {.x = fmin(p1.x, p2.x),
                     .y = fmin(p1.y, p2.y)};

    vector3_t max = {.x = fmax(p1.x, p2.x),
                     .y = fmax(p1.y, p2.y)};
    IntersectionType_E ret = NO_INTERSECTION;

    const double EPSILON = 1e-6;
    if (max.x - min.x < EPSILON && fabs(sol1.x - min.x) < EPSILON) {
        if (sol1.y >= min.y && sol1.y <= max.y) {
            sol1.x += center.x;
            sol1.y += center.y;
            *out1 = sol1;
            ret++;
        }
    } else if (max.y - min.y < EPSILON && fabs(sol1.y - min.y) < EPSILON) {
        if (sol1.x >= min.x && sol1.x <= max.x) {
            sol1.x += center.x;
            sol1.y += center.y;
            *out1 = sol1;
            ret++;
        }
    } else if (sol1.x >= min.x && sol1.x <= max.x && sol1.y >= min.y && sol1.y <= max.y) {
        sol1.x += center.x;
        sol1.y += center.y;
        *out1 = sol1;
        ret++;
    }

    if (max.x - min.x < EPSILON && fabs(sol2.x - min.x) < EPSILON) {
        if (sol2.y >= min.y && sol2.y <= max.y) {
            sol2.x += center.x;
            sol2.y += center.y;
            if (ret == NO_INTERSECTION)
                *out1 = sol2;
            else
                *out2 = sol2;
            ret++;
        }
    } else if (max.y - min.y < EPSILON && fabs(sol2.y - min.y) < EPSILON) {
        if (sol2.x >= min.x && sol2.x <= max.x) {
            sol2.x += center.x;
            sol2.y += center.y;
            if (ret == NO_INTERSECTION)
                *out1 = sol2;
            else
                *out2 = sol2;
            ret++;
        }
    } else if (sol2.x >= min.x && sol2.x <= max.x && sol2.y >= min.y && sol2.y <= max.y) {
        sol2.x += center.x;
        sol2.y += center.y;
        if (sol1.x == sol2.x && sol1.y == sol2.y) {
            return ONE_INTERSECTION;
        }
        if (ret == NO_INTERSECTION)
            *out1 = sol2;
        else
            *out2 = sol2;
        ret++;
    }
    return ret;
}

vector3_t pickClosestIntersection(vector3_t targetPoint, vector3_t p1, vector3_t p2) {
    if (isnan(p1.x) && isnan(p1.y))
        return p2;
    if (isnan(p2.x) && isnan(p2.y))
        return p1;
    double d1 = dist(targetPoint.x, targetPoint.y, p1.x, p1.y);
    double d2 = dist(targetPoint.x, targetPoint.y, p2.x, p2.y);
    if (d1 < d2)
        return p1;
    return p2;
}

void printMatrix(arm_matrix_instance_f32 mat) {
    for (int i = 0; i < mat.numRows; i++) {
        for (int j = 0; j < mat.numCols; j++) {
            uprintf("%.3f ", mat.pData[i * mat.numCols + j]);
        }
        uprintf("\n");
    }
}

void printVector(float32_t *vec, size_t len) {
    for (int i = 0; i < len; i++) {
        uprintf("%.3f ", vec[i]);
    }
    uprintf("\n");
}