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

void quat_mult(Quaternion p, Quaternion q, Quaternion result)
{
	result[0] = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
	result[1] = p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2];
	result[2] = p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1];
	result[3] = p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0];
}

void conjugate(Quaternion q, Vector3f vec, Vector3f result)
{
	Quaternion vec_q, res, q_inv;
	
	quat_inverse(q, q_inv);
	vec_q[0] = 0;	
	vec_q[1] = vec[0];
	vec_q[2] = vec[1];
	vec_q[3] = vec[2];
	quat_mult(q, vec_q, res);
	quat_mult(res, q_inv, res);
    
	result[0] = res[1];
	result[1] = res[2];
	result[2] = res[3];
}

void conjugate_inversed(Quaternion q, Vector3f vec, Vector3f result)
{	
	Quaternion vec_q, res, q_inv;
	
	quat_inverse(q, q_inv);
	vec_q[0] = 0;	
	vec_q[1] = vec[0];
	vec_q[2] = vec[1];
	vec_q[3] = vec[2];
	
	quat_mult(q_inv, vec_q, res);
	quat_mult(res, q, res);
    
	result[0] = res[1];
	result[1] = res[2];
	result[2] = res[3];
}

void derivative(Quaternion q, Vector3f vec, Quaternion result)
{
	Quaternion vec_q;
	
	vec_q[0] = 0;	
	vec_q[1] = vec[0];
	vec_q[2] = vec[1];
	vec_q[3] = vec[2];
	
	quat_mult(q, vec_q, result);
	
	result[0] *= 0.5;
	result[1] *= 0.5;
	result[2] *= 0.5;
	result[3] *= 0.5;
}

void quat_inverse(Quaternion q, Quaternion result)
{
	float normSq = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    
	result[0] = q[0] / normSq;
	result[1] = -q[1] / normSq;
	result[2] = -q[2] / normSq;
	result[3] = -q[3] / normSq;
}
void quat_normalize(Quaternion q)
{
	float norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    
	q[0] = q[0] / norm;
	q[1] = q[1] / norm;
	q[2] = q[2] / norm;
	q[3] = q[3] / norm;
}

float wrap_pi(float x)
{
	int c = 0;

	while (x >= M_PI) {
		x -= 2 * M_PI;

		if (c++ > 100) {
			return 999999;
		}
	}

	c = 0;

	while (x < -M_PI) {
		x += 2 * M_PI;

		if (c++ > 100) {
			return 999999;
		}
	}

	return x;
}



void getQuaternion(Dcmf dcm, Quaternion result) {
    
	float tr = dcm[0][0] + dcm[1][1] + dcm[2][2];

	if (tr > 0) { 
		float S = sqrt(tr + 1.0) * 2;  // S=4*qw 
		result[0] = 0.25 * S;
		result[1] = (dcm[2][1] - dcm[1][2]) / S;
		result[2] = (dcm[0][2] - dcm[2][0]) / S; 
		result[3] = (dcm[1][0] - dcm[0][1]) / S; 
	}
	else if ((dcm[0][0] > dcm[1][1]) && (dcm[0][0] > dcm[2][2])) { 
		float S = sqrt(1.0 + dcm[0][0] - dcm[1][1] - dcm[2][2]) * 2;  // S=4*qx 
		result[0] = (dcm[2][1] - dcm[1][2]) / S;
		result[1] = 0.25 * S;
		result[2] = (dcm[0][1] + dcm[1][0]) / S; 
		result[3] = (dcm[0][2] + dcm[2][0]) / S; 
	}
	else if (dcm[1][1] > dcm[2][2]) { 
		float S = sqrt(1.0 + dcm[1][1] - dcm[0][0] - dcm[2][2]) * 2;  // S=4*qy
		result[0] = (dcm[0][2] - dcm[2][0]) / S;
		result[1] = (dcm[0][1] + dcm[1][0]) / S; 
		result[2] = 0.25 * S;
		result[3] = (dcm[1][2] + dcm[2][1]) / S; 
	}
	else { 
		float S = sqrt(1.0 + dcm[2][2] - dcm[0][0] - dcm[1][1]) * 2;  // S=4*qz
		result[0] = (dcm[1][0] - dcm[0][1]) / S;
		result[1] = (dcm[0][2] + dcm[2][0]) / S;
		result[2] = (dcm[1][2] + dcm[2][1]) / S;
		result[3] = 0.25 * S;
	}
}


void slerp(Quaternion q1, Quaternion q2, float k, Quaternion result) {
	float cosOmega = q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
    
	if (cosOmega >= 1)
		cosOmega = 0.99999;
		else 
		if (cosOmega <= 0)
			cosOmega = 0.00001;

	float omega = acos(cosOmega);
	Quaternion t1, t2;
	t1[0] = q1[0] * sin((1 - k) * omega);
	t1[1] = q1[1] * sin((1 - k) * omega);
	t1[2] = q1[2] * sin((1 - k) * omega);
	t1[3] = q1[3] * sin((1 - k) * omega);
	
	t2[0] = q2[0] * sin(k * omega);
	t2[1] = q2[1] * sin(k * omega);
	t2[2] = q2[2] * sin(k * omega);
	t2[3] = q2[3] * sin(k * omega);
	
	result[0] = (t1[0] + t2[0]) / sin(omega);
	result[1] = (t1[2] + t2[1]) / sin(omega);
	result[2] = (t1[2] + t2[2]) / sin(omega);
	result[3] = (t1[3] + t2[3]) / sin(omega);
  }

void getRotationMatrix(Quaternion q, Dcmf result) {
    
    float sqw = q[0]*q[0];
    float sqx = q[1]*q[1];
    float sqy = q[2]*q[2];
    float sqz = q[3]*q[3];

    // invs (inverse square length) is only required if quaternion is not already normalised
    float invs = 1 / (sqx + sqy + sqz + sqw);
    result[0][0] = ( sqx - sqy - sqz + sqw)*invs; // since sqw + sqx + sqy + sqz =1/invs*invs
    result[1][1] = (-sqx + sqy - sqz + sqw)*invs;
    result[2][2] =  (-sqx - sqy + sqz + sqw)*invs;
    
    float tmp1 = q[1]*q[2];
    float tmp2 = q[3]*q[0];
    result[1][0] = 2.0 * (tmp1 + tmp2)*invs;
    result[0][1] =  2.0 * (tmp1 - tmp2)*invs;
    
    tmp1 = q[1]*q[3];
    tmp2 = q[2]*q[0];
    result[2][0] =  2.0 * (tmp1 - tmp2)*invs ;
    result[0][2] =  2.0 * (tmp1 + tmp2)*invs ;
    tmp1 = q[2]*q[3];
    tmp2 = q[1]*q[0];
    result[2][1] =  2.0 * (tmp1 + tmp2)*invs ;
    result[1][2] =  2.0 * (tmp1 - tmp2)*invs ;
  }