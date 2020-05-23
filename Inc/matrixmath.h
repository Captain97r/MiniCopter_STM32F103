#ifndef __MATRIXMATH_H
#define __MATRIXMATH_H

#include "stm32f1xx_hal.h"

typedef float Vector3f[3];
typedef float Dcmf[3][3];

float vector_length(Vector3f vec);
void vector_normalize(Vector3f vec);
void vector_substract(Vector3f arg1, Vector3f arg2, Vector3f result);
void vector_mult(Vector3f arg1, Vector3f arg2, Vector3f result);
void vector_cross(Vector3f arg1, Vector3f arg2, Vector3f result);

void matrix_set_row(Dcmf* matrix, Vector3f* vec, uint8_t row);
void matrix_multiply(Dcmf* arg1, Dcmf* arg2, uint8_t m, uint8_t n, Dcmf* result);

#endif /* __MATRIXMATH_H */
