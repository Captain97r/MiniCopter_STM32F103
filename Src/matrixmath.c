#include "matrixmath.h"
#include "math.h"

float vector_length(Vector3f vec)
{
	return sqrt(vec[0] * vec[0] +  vec[1] * vec[1] + vec[2] * vec[2]);
}

void vector_normalize(Vector3f vec)
{
	float sqr = sqrt(vec[0] * vec[0] +  vec[1] * vec[1] + vec[2] * vec[2]);
	vec[0] /= sqr;
	vec[1] /= sqr;
	vec[2] /= sqr;
}

void vector_substract(Vector3f arg1, Vector3f arg2, Vector3f result)
{
	result[0] = arg1[0] - arg2[0];
	result[1] = arg1[1] - arg2[1];
	result[2] = arg1[2] - arg2[2];
}

void vector_mult(Vector3f arg1, Vector3f arg2, Vector3f result)
{
	result[0] = arg1[0] * arg2[0];
	result[1] = arg1[1] * arg2[1];
	result[2] = arg1[2] * arg2[2];
}

void vector_cross(Vector3f arg1, Vector3f arg2, Vector3f result)
{
	result[0] = arg1[1]*arg2[2] - arg1[2]*arg2[1];
	result[1] = -arg1[0]*arg2[2] + arg1[2]*arg2[0];
	result[2] = arg1[0]*arg2[1] - arg1[1]*arg2[0];
}

void matrix_set_row(Dcmf* matrix, Vector3f* vec, uint8_t row)
{
	*(matrix[row][0]) = *(vec[0]);
	*(matrix[row][0]) = *(vec[1]);
	*(matrix[row][0]) = *(vec[2]);
}


void matrix_multiply(Dcmf* arg1, Dcmf* arg2, uint8_t m, uint8_t n, Dcmf* result)
{
	Dcmf res;
	for (uint8_t i = 0; i < m; i++)
		for (uint8_t j = 0; j < n; j++)
			for (uint8_t k = 0; k < m; k++)
				(*result)[i][j] += (*arg1)[i][k] * (*arg2)[k][j];
}