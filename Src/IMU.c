#include "IMU.h"
#include "math.h"
#include "matrixmath.h"

#define Kp 1.0f
#define Ki 0.0f

#define SAMPLE_FREQ_HZ	200.0f		// sample frequency in Hz
#define BETA_DEF		0.0001f		// 2 * proportional gain


float beta = BETA_DEF;  								// 2 * proportional gain (Kp)
float delta_t = 1 / SAMPLE_FREQ_HZ; 					// integration interval for both filter schemes

unsigned long _last_time = 0;
uint8_t inited = 0;

// for PX4 algo
float		_mag_decl		= -0.193f;
float		_w_accel		= 3.0f;
float		_w_mag			= 0.0f;
float		_w_gyro_bias	= 0.1f;
float		_bias_max		= 0.05f;

Vector3f _gyro_bias;

Dcmf rm;

void MadgwickAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float ax, float ay, float az, float gx, float gy, float gz);
void SimpleUpdateIMU(float ax, float ay, float az, float gx, float gy, float gz);
void PX4_altitude_estimator_c_update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void complementary_filter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

void handleAccelData(Vector3f acc, Vector3f mag, Dcmf result);

void handleGyroData(Vector3f gyr, float t, Dcmf result);

void toEuler();
float invSqrt(float x);

void calculate_orientation()
{
	MPU9250_get_accel();
	MPU9250_get_gyro();
	MPU9250_get_mag();
	MPU9250_get_temp();
	
	complementary_filter(mpu9250.accelerometer.data.x,
		mpu9250.accelerometer.data.y,
		mpu9250.accelerometer.data.z,
									mpu9250.gyroscope.data.x,		mpu9250.gyroscope.data.y,		mpu9250.gyroscope.data.z,
									mpu9250.magnetometer.data.x,	 mpu9250.magnetometer.data.y,	mpu9250.magnetometer.data.z);
	
//	MadgwickAHRSupdate(mpu9250.accelerometer.data.x,	mpu9250.accelerometer.data.y,	mpu9250.accelerometer.data.z,
//					   mpu9250.gyroscope.data.x,		mpu9250.gyroscope.data.y,		mpu9250.gyroscope.data.z,
//					   mpu9250.magnetometer.data.x,		mpu9250.magnetometer.data.y,	mpu9250.magnetometer.data.z);
	
	toEuler();
}

void PX4_altitude_estimator_c_init(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	
	_last_time = HAL_GetTick();
	inited = 1;
//	Vector3f acc = { ax, ay, az };
//	Vector3f gyr = { gx, gy, gz };
//	Vector3f mag = { mx, my, mz };
//	
//	if (vector_length(acc) > 20.0f || vector_length(gyr) > 5.0f || vector_length(mag) > 75.0f)
//		return;
//	
//	// Rotation matrix can be easily constructed from acceleration and mag field vectors
//	// 'k' is Earth Z axis (Down) unit vector in body frame
//	Vector3f k = { -ax, -ay, -az };
//	vector_normalize(k);
//	
//	// 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
//	vector_normalize(mag);
//	
//	Vector3f t1, t2, i;
//	vector_mult(mag, k, t1);
//	vector_mult(k, t1, t2);
//	vector_substract(mag, t2, i);
//	vector_normalize(i);
//	
//	// 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
//	Vector3f j;
//	vector_cross(k, i, j);
//	
//	float head = atan2(mag[1], mag[0]) * 180 / 3.14;
//	
//	// Fill rotation matrix
//	Dcmf R;
//	matrix_set_row(&R, &i, 0);	
//	matrix_set_row(&R, &j, 1);
//	matrix_set_row(&R, &k, 2);
}

void PX4_altitude_estimator_c_update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	if (inited == 0)
	{
		PX4_altitude_estimator_c_init(ax, ay, az, gx, gy, gz, mx, my, mz);
		return;
	}
	
	float dt = (HAL_GetTick() - _last_time) * 0.001;
	_last_time = HAL_GetTick();
	
	Vector3f acc = { ax, ay, az };
	Vector3f gyr = { gx, gy, gz };
	Vector3f mag = { mx, my, mz };
	
	Quaternion q;
	q[0] = copter.orientation.quat.w;
	q[1] = copter.orientation.quat.x;
	q[2] = copter.orientation.quat.y;
	q[3] = copter.orientation.quat.z;
	
	// Angular rate of correction
    Vector3f corr;
    
	float spinRate = sqrt(gx * gx + gy * gy + gz * gz);
    
	// Project mag field vector to global frame and extract XY component
	Vector3f mag_earth;
	conjugate(q, mag, mag_earth);
	float mag_err = wrap_pi(atan2(mag_earth[1], mag_earth[0]) - _mag_decl);
	float gainMult = 1.0f;
	float fifty_dps = 0.873f;

	//if (spinRate > fifty_dps) {
	//  gainMult = math::min(spinRate / fifty_dps, 10.0f);
	//}

	// Project magnetometer correction to body frame
	Vector3f m, corr_m;
	m[0] = 0;
	m[1] = 0;
	m[2] = -mag_err;
    
	conjugate_inversed(q, m, corr_m);
	for (int i = 0; i < 3; i++)
		corr[i] += corr_m[i] * _w_mag * gainMult;
    
	quat_normalize(q);
    
	// Accelerometer correction
	// Project 'k' unit vector of earth frame to body frame
	//float[] t = new float[3];
	//t[2] = -1.0f;
	//float[] k = _q.conjugate_inversed(t);
	// Optimized version with dropped zeros
	Vector3f k;
    
	k[0] = -2.0f * (q[1] * q[3] - q[0] * q[2]);
	k[1] = -2.0f * (q[2] * q[3] + q[0] * q[1]);
	k[2] = -(q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
    
  
	// If we are not using acceleration compensation based on GPS velocity,
	// fuse accel data only if its norm is close to 1 g (reduces drift).
	float accel_norm_sq = ax * ax + ay * ay + az * az;
	float upper_accel_limit = 9.81 * 1.1f;
	float lower_accel_limit = 9.81 * 0.9f;
  
	if (((accel_norm_sq > lower_accel_limit * lower_accel_limit) &&
	          (accel_norm_sq < upper_accel_limit * upper_accel_limit))) {
  
		Vector3f acc_norm;
		for (int i = 0; i < 3; i++) 
			acc_norm[i] = acc[i] / sqrt(accel_norm_sq);
       
        
        
		Vector3f corr_a;
		vector_cross(k, acc_norm, corr_a);
		for (int i = 0; i < 3; i++) 
			corr[i] += corr_a[i] * _w_accel; 
	}
  
  
	// Gyro bias estimation
	if(spinRate < 0.175f) {
		for (int i = 0; i < 3; i++) {
			_gyro_bias[i] += corr[i] * (_w_gyro_bias * dt);
  
			if (_gyro_bias[i] > _bias_max)
				_gyro_bias[i] = _bias_max;
			if (_gyro_bias[i] < -_bias_max)
				_gyro_bias[i] = -_bias_max;
		}
  
	}
    
	Vector3f rates;
    
	for (int i = 0; i < 3; i++)
		rates[i] = gyr[i] + _gyro_bias[i];
  
	// Feed forward gyro
	for(int i = 0 ; i < 3 ; i++)
	  corr[i] += rates[i];
  
	Quaternion q_derivative;
	derivative(q, corr, q_derivative);
	// Apply correction to state
	for(int i = 0 ; i < 4 ; i++)
	  q[i] += q_derivative[i] * dt;
  
	// Normalize quaternion
	quat_normalize(q);
	
	copter.orientation.quat.w = q[0];
	copter.orientation.quat.x = q[1];
	copter.orientation.quat.y = q[2];
	copter.orientation.quat.z = q[3];
}

void complementary_filter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	
	if (inited == 0)
	{
		rm[0][0] = 1.0f;
		rm[1][1] = 1.0f;
		rm[2][2] = 1.0f;
		
		inited = 1;
		return;
	}
	
	Vector3f acc = { ax, ay, az };
	Vector3f gyr = { gx, gy, gz };
	Vector3f mag = { mx, my, mz };
	
	Dcmf rmAcc, rmGyr, rmComp;
	Quaternion qAcc, qGyr, qComp;
	
	handleAccelData(acc, mag, rmAcc);
	handleGyroData(gyr, 5, rmGyr);
	
	rmComp[0][0] = rm[0][0];
	rmComp[0][1] = rm[0][1];
	rmComp[0][2] = rm[0][2];
	rmComp[1][0] = rm[1][0];
	rmComp[1][1] = rm[1][1];
	rmComp[1][2] = rm[1][2];
	rmComp[2][0] = rm[2][0];
	rmComp[2][1] = rm[2][1];
	rmComp[2][2] = rm[2][2];
		
	getQuaternion(rmAcc, qAcc);
	getQuaternion(rmGyr, qGyr);
	getQuaternion(rmComp, qComp);
    
	float Beta = 2.5f;
	Quaternion result, r1;
	slerp(qComp, qGyr, Beta * 0.01, result);
	slerp(result, qAcc, Beta * 0.01, r1);
	
	copter.orientation.quat.w = r1[0];
	copter.orientation.quat.x = r1[1];
	copter.orientation.quat.y = r1[2];
	copter.orientation.quat.z = r1[3];
    
	getRotationMatrix(r1, rm);
}


void handleAccelData(Vector3f acc, Vector3f mag, Dcmf result) {    
	float ax = acc[0];
	float ay = acc[1];
	float az = acc[2];
    
	float ex = mag[0];
	float ey = mag[1];
	float ez = mag[2];
    
	float hx = ey * az - ez * ay;
	float hy = ez * ax - ex * az;
	float hz = ex * ay - ey * ax;
    
	float invh = 1 / sqrt(hx * hx + hy * hy + hz * hz);
	hx *= invh;
	hy *= invh;
	hz *= invh;
    
	float inva = 1 / sqrt(ax * ax + ay * ay + az * az);
	ax *= inva;
	ay *= inva;
	az *= inva;
    
	float mx = ay * hz - az * hy;
	float my = az * hx - ax * hz;
	float mz = ax * hy - ay * hx;
    
	result[0][0] = hx;
	result[0][1] = hy;
	result[0][2] = hz;
	result[1][0] = mx;
	result[1][1] = my;
	result[1][2] = mz;
	result[2][0] = ax;
	result[2][1] = ay;
	result[2][2] = az;
}

void handleGyroData(Vector3f gyr, float t, Dcmf result) {
	gyr[0] *= 0.001 * t;
	gyr[1] *= 0.001 * t;
	gyr[2] *= 0.001 * t;
    
	Dcmf rot;
	rot[0][0] = 1;
	rot[0][1] = -gyr[2];
	rot[0][2] = gyr[1];
	rot[1][0] = gyr[2];
	rot[1][1] = 1;
	rot[1][2] = -gyr[0];
	rot[2][0] = -gyr[1];
	rot[2][1] = gyr[0];
	rot[2][2] = 1;
	
	for (int i = 0; i < 3; i++) {
		for (int j = 0; j < 3; j++) {
			float temp = 0;
			for (int k = 0; k < 3; k++) {
				temp += rm[i][k] * rot[k][j];
			}
			result[i][j] = temp;
		}
	}
	rm[0][0] = result[0][0];
	rm[0][1] = result[0][1];
	rm[0][2] = result[0][2];  
	rm[1][0] = result[1][0];
	rm[1][1] = result[1][1];
	rm[1][2] = result[1][2];  
	rm[2][0] = result[2][0];
	rm[2][1] = result[2][1];
	rm[2][2] = result[2][2];    
}

void MadgwickAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz) {
	
	float q0 = copter.orientation.quat.w, q1 = copter.orientation.quat.x, q2 = copter.orientation.quat.y, q3 = copter.orientation.quat.z;
	
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q0 * mx;
		_2q0my = 2.0f * q0 * my;
		_2q0mz = 2.0f * q0 * mz;
		_2q1mx = 2.0f * q1 * mx;
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_2q0q2 = 2.0f * q0 * q2;
		_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q0 * q0;
		q0q1 = q0 * q1;
		q0q2 = q0 * q2;
		q0q3 = q0 * q3;
		q1q1 = q1 * q1;
		q1q2 = q1 * q2;
		q1q3 = q1 * q3;
		q2q2 = q2 * q2;
		q2q3 = q2 * q3;
		q3q3 = q3 * q3;

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * delta_t;
	q1 += qDot2 * delta_t;
	q2 += qDot3 * delta_t;
	q3 += qDot4 * delta_t;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	copter.orientation.quat.w = q0 * recipNorm;
	copter.orientation.quat.x = q1 * recipNorm;
	copter.orientation.quat.y = q2 * recipNorm;
	copter.orientation.quat.z = q3 * recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float ax, float ay, float az, float gx, float gy, float gz) {
	
	float q0 = copter.orientation.quat.w, q1 = copter.orientation.quat.x, q2 = copter.orientation.quat.y, q3 = copter.orientation.quat.z;
	
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2, _8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * delta_t;
	q1 += qDot2 * delta_t;
	q2 += qDot3 * delta_t;
	q3 += qDot4 * delta_t;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	copter.orientation.quat.w = q0 * recipNorm;
	copter.orientation.quat.x = q1 * recipNorm;
	copter.orientation.quat.y = q2 * recipNorm;
	copter.orientation.quat.z = q3 * recipNorm;
}

void toEuler() {
	// roll (x-axis rotation)
	float sinr_cosp = 2 * (copter.orientation.quat.w * copter.orientation.quat.x + copter.orientation.quat.y * copter.orientation.quat.z);
	float cosr_cosp = 1 - 2 * (copter.orientation.quat.x * copter.orientation.quat.x + copter.orientation.quat.y * copter.orientation.quat.y);
	copter.orientation.euler.roll = atan2(sinr_cosp, cosr_cosp) * 180 / M_PI;

	// pitch (y-axis rotation)
	float sinp = 2 * (copter.orientation.quat.w * copter.orientation.quat.y - copter.orientation.quat.z * copter.orientation.quat.x);
	if (sinp >= 1 || sinp <= -1)
		copter.orientation.euler.pitch = copysign(M_PI / 2, sinp) * 180 / M_PI;  // use 90 degrees if out of range
	else
	    copter.orientation.euler.pitch = asin(sinp) * 180 / M_PI;

	// yaw (z-axis rotation)
	float siny_cosp = 2 * (copter.orientation.quat.w * copter.orientation.quat.z + copter.orientation.quat.x * copter.orientation.quat.y);
	float cosy_cosp = 1 - 2 * (copter.orientation.quat.y * copter.orientation.quat.y + copter.orientation.quat.z * copter.orientation.quat.z);
	copter.orientation.euler.yaw = atan2(siny_cosp, cosy_cosp) * 180 / M_PI;
}


float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}