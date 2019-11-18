#include "IMU.h"
#include "math.h"

#define Kp 2.0f * 5.0f
#define Ki 0.0f

#define SAMPLE_FREQ_HZ	200.0f		// sample frequency in Hz
#define BETA_DEF		0.1f		// 2 * proportional gain


float beta = BETA_DEF;  								// 2 * proportional gain (Kp)

float delta_t = 1 / SAMPLE_FREQ_HZ; 					// integration interval for both filter schemes

float eInt[3] = { 0.0f, 0.0f, 0.0f }; 					// vector to hold integral error for Mahony method
float GyroMeasError = M_PI * (60.0f / 180.0f);
float GyroMeasDrift = M_PI * (1.0f / 180.0f); 			// gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)

void MadgwickAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float ax, float ay, float az, float gx, float gy, float gz);
void toEuler();
float invSqrt(float x);

void calculate_orientation()
{
	MPU9250_get_accel();
	MPU9250_get_gyro();
	MPU9250_get_mag();
	MPU9250_get_temp();
//	MadgwickAHRSupdate(mpu9250.accelerometer.data.x, mpu9250.accelerometer.data.y,  mpu9250.accelerometer.data.z,
//					   mpu9250.gyroscope.data.x,	 mpu9250.gyroscope.data.y,		mpu9250.gyroscope.data.z,
//					   mpu9250.magnetometer.data.x,	 mpu9250.magnetometer.data.y,	mpu9250.magnetometer.data.z);
	
	MadgwickAHRSupdateIMU(	mpu9250.accelerometer.data.x,	mpu9250.accelerometer.data.y,	mpu9250.accelerometer.data.z,
							mpu9250.gyroscope.data.x,		mpu9250.gyroscope.data.y,		mpu9250.gyroscope.data.z);
	
	toEuler();
	float mx = mpu9250.magnetometer.data.x;
	float my = mpu9250.magnetometer.data.y;
	float mz = mpu9250.magnetometer.data.z;
	copter.orientation.heading = atan2f(my * cos(copter.orientation.euler.roll) + mz * sin(copter.orientation.euler.roll), 
		mx * cos(copter.orientation.euler.pitch) + my * sin(copter.orientation.euler.roll) * sin(copter.orientation.euler.pitch) - mz * cos(copter.orientation.euler.roll) * sin(copter.orientation.euler.pitch));
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
	float q1 = copter.orientation.quat.w, q2 = copter.orientation.quat.x, q3 = copter.orientation.quat.y, q4 = copter.orientation.quat.z;
	float test = q2*q3 + q4*q1;
	if (test > 0.499) {
		copter.orientation.euler.yaw = 2 * atan2(q2, q1) * 180 / M_PI;
		copter.orientation.euler.pitch = M_PI / 2 * 180 / M_PI;
		copter.orientation.euler.roll = 0;
		return;
	}
	if (test < -0.499) {
		copter.orientation.euler.yaw = -2 * atan2(q2, q1) * 180 / M_PI;
		copter.orientation.euler.pitch = -M_PI / 2 * 180 / M_PI;
		copter.orientation.euler.roll = 0;
		return;
	}
	float sqx = q2*q2;
	float sqy = q3*q3;
	float sqz = q4*q4;
	copter.orientation.euler.yaw = atan2(2*q3*q1 - 2*q2*q4, 1 - 2*sqy - 2*sqz) * 180 / M_PI;
	copter.orientation.euler.pitch = asin(2*test) * 180 / M_PI;
	copter.orientation.euler.roll = atan2(2*q2*q1 - 2*q3*q4, 1 - 2*sqx - 2*sqz) * 180 / M_PI;
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