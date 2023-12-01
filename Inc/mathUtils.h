#ifndef __MATHUTILS_H__
#define __MATHUTILS_H__
#include "arm_math.h"

/**
 * @brief 3D float vector. Can access individual components or as an array.
 */
typedef union vector3_t {
    double v[3];
    struct {
        double x;
        double y;
        double z;
    };
} vector3_t;

typedef enum {
    NO_INTERSECTION = 0,
    ONE_INTERSECTION = 1,
    TWO_INTERSECTIONS = 2,
} IntersectionType_E;

double sign(double x);

double map(double x, double a, double b, double c, double d);
double dist(double x1, double y1, double x2, double y2);
double clamp(double x, double a, double b);
double adjustTurn(double angle);
IntersectionType_E circleSegmentIntersection(vector3_t center, double radius, vector3_t p1, vector3_t p2, vector3_t *out1, vector3_t *out2);
vector3_t pickClosestIntersection(vector3_t targetPoint, vector3_t p1, vector3_t p2);
void printMatrix(arm_matrix_instance_f32 mat);
void printVector(float32_t *vec, size_t len);

#define RAD_TO_DEG(x) ((x)*180 / PI)
#define DEG_TO_RAD(x) ((x)*PI / 180)
// typedef struct vector3_t {
//     float x;
//     float y;
//     float z;
// } vector3_t;

#endif  // __MATHUTILS_H__