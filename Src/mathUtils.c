#include "mathUtils.h"
#include "arm_math.h"
#include "debug.h"

double map(double x, double a, double b, double c, double d) {
    return (x - a) / (b - a) * (d - c) + c;
}

double dist(double x1, double y1, double x2, double y2) {
    return sqrt((x2- x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
}

void printMatrix(arm_matrix_instance_f32 mat){
    for (int i = 0; i < mat.numRows; i++){
        for (int j = 0; j < mat.numCols; j++){
            uprintf("%.3f ", mat.pData[i * mat.numCols + j]);
        }
        uprintf("\n");
    }
}

void printVector(float32_t *vec, size_t len){
    for (int i = 0; i < len; i++){
        uprintf("%.3f ", vec[i]);
    }
    uprintf("\n");
}