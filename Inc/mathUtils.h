#ifndef __MATHUTILS_H__
#define __MATHUTILS_H__

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

double map(double x, double a, double b, double c, double d);
// typedef struct vector3_t {
//     float x;
//     float y;
//     float z;
// } vector3_t;

#endif  // __MATHUTILS_H__