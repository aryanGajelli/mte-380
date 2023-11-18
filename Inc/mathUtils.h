#ifndef __MATHUTILS_H__
#define __MATHUTILS_H__
#include "arm_math.h"
/**
 * @brief 3D float vector. Can access individual components or as an array.
 */
typedef union vector3_t {
    float v[3];
    struct {
        float x;
        float y;
        float z;
    };
} vector3_t;

double sign(double x);

double map(double x, double a, double b, double c, double d);
double dist(double x1, double y1, double x2, double y2);
double clamp(double x, double a, double b);
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