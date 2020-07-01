#ifndef __MATRIXMATH_H
#define __MATRIXMATH_H

#include "stm32f1xx_hal.h"

typedef float Vector3f[3];
typedef float Quaternion[4];
typedef float Dcmf[3][3];

float vector_length(Vector3f vec);
void vector_normalize(Vector3f vec);
void vector_substract(Vector3f arg1, Vector3f arg2, Vector3f result);
void vector_mult(Vector3f arg1, Vector3f arg2, Vector3f result);
void vector_cross(Vector3f arg1, Vector3f arg2, Vector3f result);

void conjugate(Quaternion q, Vector3f vec, Quaternion result);
void conjugate_inversed(Quaternion q, Vector3f vec, Quaternion result);
void derivative(Quaternion q, Vector3f vec, Quaternion result);
void quat_inverse(Quaternion q, Quaternion result);
void quat_normalize(Quaternion q);
void quat_mult(Quaternion p, Quaternion q, Quaternion result);

void matrix_set_row(Dcmf* matrix, Vector3f* vec, uint8_t row);
void matrix_multiply(Dcmf* arg1, Dcmf* arg2, uint8_t m, uint8_t n, Dcmf* result);

float wrap_pi(float x);


void getQuaternion(Dcmf dcm, Quaternion result);

void slerp(Quaternion q1, Quaternion q2, float k, Quaternion result);

void getRotationMatrix(Quaternion q, Dcmf result);

#endif /* __MATRIXMATH_H */
