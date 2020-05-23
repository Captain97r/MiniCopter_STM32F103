#include "IMU.h"
#include "math.h"
#include "matrixmath.h"

#define Kp 1.0f
#define Ki 0.0f

#define SAMPLE_FREQ_HZ	200.0f		// sample frequency in Hz
#define BETA_DEF		0.0001f		// 2 * proportional gain


float beta = BETA_DEF;  								// 2 * proportional gain (Kp)
float delta_t = 1 / SAMPLE_FREQ_HZ; 					// integration interval for both filter schemes

// for PX4 algo
float		_w_accel = 0.0f;
float		_w_mag = 0.0f;
float		_w_ext_hdg = 0.0f;
float		_w_gyro_bias = 0.0f;
float		_mag_decl = 0.0f;
float		_bias_max = 0.0f;

void MadgwickAHRSupdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void MadgwickAHRSupdateIMU(float ax, float ay, float az, float gx, float gy, float gz);
void SimpleUpdateIMU(float ax, float ay, float az, float gx, float gy, float gz);
void PX4_altitude_estimator_c(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);

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
	
	//SimpleUpdateIMU(	mpu9250.accelerometer.data.x, mpu9250.accelerometer.data.y, mpu9250.accelerometer.data.z,
	//						mpu9250.gyroscope.data.x,		mpu9250.gyroscope.data.y,		mpu9250.gyroscope.data.z);
	
	PX4_altitude_estimator_c(mpu9250.accelerometer.data.x, mpu9250.accelerometer.data.y, mpu9250.accelerometer.data.z,
							 mpu9250.gyroscope.data.x,	 mpu9250.gyroscope.data.y,		mpu9250.gyroscope.data.z,
							 mpu9250.magnetometer.data.x,	 mpu9250.magnetometer.data.y,	mpu9250.magnetometer.data.z);
	
	//toEuler();
	float mx = mpu9250.magnetometer.data.x;
	float my = mpu9250.magnetometer.data.y;
	float mz = mpu9250.magnetometer.data.z;
	//copter.orientation.heading = atan2f(my * cos(copter.orientation.euler.roll) + mz * sin(copter.orientation.euler.roll), 
	//	mx * cos(copter.orientation.euler.pitch) + my * sin(copter.orientation.euler.roll) * sin(copter.orientation.euler.pitch) - mz * cos(copter.orientation.euler.roll) * sin(copter.orientation.euler.pitch));
}


//void PX4_attitude_estimator(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
//{
//	/// Init
//	
//	// Rotation matrix can be easily constructed from acceleration and mag field vectors
//	// 'k' is Earth Z axis (Down) unit vector in body frame
//	Vector3f k = -_accel;
//	k.normalize();
//
//	// 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
//	Vector3f i = (_mag - k * (_mag * k));
//	i.normalize();
//
//	// 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
//	Vector3f j = k % i;
//
//	// Fill rotation matrix
//	Dcmf R;
//	R.setRow(0, i);
//	R.setRow(1, j);
//	R.setRow(2, k);
//
//	// Convert to quaternion
//	_q = R;
//
//	// Compensate for magnetic declination
//	Quatf decl_rotation = Eulerf(0.0f, 0.0f, _mag_decl);
//	_q = _q * decl_rotation;
//
//	_q.normalize();
//
//	/// Update
//	
//	Vector3f corr;
//	float spinRate = _gyro.length();
//
//	// Magnetometer correction
//	// Project mag field vector to global frame and extract XY component
//	Vector3f mag_earth = _q.conjugate(_mag);
//	float mag_err = wrap_pi(atan2f(mag_earth(1), mag_earth(0)) - _mag_decl);
//	float gainMult = 1.0f;
//	const float fifty_dps = 0.873f;
//
//	if (spinRate > fifty_dps) {
//		gainMult = math::min(spinRate / fifty_dps, 10.0f);
//	}
//
//	// Project magnetometer correction to body frame
//	corr += _q.conjugate_inversed(Vector3f(0.0f, 0.0f, -mag_err)) * _w_mag * gainMult;
//
//	_q.normalize();
//
//	// Accelerometer correction
//	// Project 'k' unit vector of earth frame to body frame
//	// Vector3f k = _q.conjugate_inversed(Vector3f(0.0f, 0.0f, 1.0f));
//	// Optimized version with dropped zeros
//	Vector3f k(
//		2.0f * (_q(1) * _q(3) - _q(0) * _q(2)),
//		2.0f * (_q(2) * _q(3) + _q(0) * _q(1)),
//		(_q(0) * _q(0) - _q(1) * _q(1) - _q(2) * _q(2) + _q(3) * _q(3)));
//
//	// If we are not using acceleration compensation based on GPS velocity,
//	// fuse accel data only if its norm is close to 1 g (reduces drift).
//	const float accel_norm_sq = _accel.norm_squared();
//	const float upper_accel_limit = 9.81 * 1.1f;
//	const float lower_accel_limit = 9.81 * 0.9f;
//
//	if (accel_norm_sq > lower_accel_limit * lower_accel_limit &&
//			  accel_norm_sq < upper_accel_limit * upper_accel_limit) {
//		corr += (k % (_accel - _pos_acc).normalized()) * _w_accel;
//	}
//
//	// Gyro bias estimation
//	if(spinRate < 0.175f) {
//		_gyro_bias += corr * (_w_gyro_bias * dt);
//
//		for (int i = 0; i < 3; i++) {
//			_gyro_bias(i) = math::constrain(_gyro_bias(i), -_bias_max, _bias_max);
//		}
//
//	}
//
//	_rates = _gyro + _gyro_bias;
//
//	// Feed forward gyro
//	corr += _rates;
//
//	// Apply correction to state
//	_q += _q.derivative1(corr) * dt;
//
//	// Normalize quaternion
//	_q.normalize();
//
//	if (!(PX4_ISFINITE(_q(0)) && PX4_ISFINITE(_q(1)) &&
//	      PX4_ISFINITE(_q(2)) && PX4_ISFINITE(_q(3)))) {
//		// Reset quaternion to last good state
//		_q = q_last;
//		_rates.zero();
//		_gyro_bias.zero();
//		return false;
//	}
//
//	//_q.zero();
//}


void PX4_altitude_estimator_c(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz)
{
	Vector3f acc = { ax, ay, az };
	Vector3f gyr = { gx, gy, gz };
	Vector3f mag = { mx, my, mz };
	
	if (vector_length(acc) > 20.0f || vector_length(gyr) > 5.0f || vector_length(mag) > 75.0f)
		return;
	
	// Rotation matrix can be easily constructed from acceleration and mag field vectors
	// 'k' is Earth Z axis (Down) unit vector in body frame
	Vector3f k = { -ax, -ay, -az };
	vector_normalize(k);
	
	// 'i' is Earth X axis (North) unit vector in body frame, orthogonal with 'k'
	vector_normalize(mag);
	
	Vector3f t1, t2, i;
	vector_mult(mag, k, t1);
	vector_mult(k, t1, t2);
	vector_substract(mag, t2, i);
	vector_normalize(i);
	
	// 'j' is Earth Y axis (East) unit vector in body frame, orthogonal with 'k' and 'i'
	Vector3f j;
	vector_cross(k, i, j);
	
	float head = atan2(mag[1], mag[0]) * 180 / 3.14;
	
	// Fill rotation matrix
	Dcmf R;
	matrix_set_row(&R, &i, 0);	
	matrix_set_row(&R, &j, 1);
	matrix_set_row(&R, &k, 2);
}

void SimpleUpdateIMU(float ax, float ay, float az, float gx, float gy, float gz)
{
	
	if (sqrt((ax*ax) + (ay*ay) + (az*az)) > 12 || sqrt((gx*gx) + (gy*gy) + (gz*gz)) > 0.1 ||
		sqrt((ax*ax) + (ay*ay) + (az*az)) < -12 || sqrt((gx*gx) + (gy*gy) + (gz*gz)) < -0.1)
		return;
	
	static uint8_t is_init = 0;
	static float gyr_pitch = 0, gyr_roll = 0;
	
  //Gyro angle calculations
  //0.0000611 = 1 / (250Hz / 65.5)
  gyr_pitch += gx * 0.00611;                       //Calculate the traveled pitch angle and add this to the angle_pitch variable
  gyr_roll += gy * 0.00611;                        //Calculate the traveled roll angle and add this to the angle_roll variable
  
  //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  gyr_pitch += gyr_roll * sin(gz * M_PI / 180);                     //If the IMU has yawed transfer the roll angle to the pitch angel
  gyr_roll -= gyr_pitch * sin(gz * M_PI / 180);                     //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  float a_total = sqrt((ax*ax) + (ay*ay) + (az*az));   //Calculate the total accelerometer vector
  //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
  float pitch_acc = asin((float)ay / a_total) * (1 / (M_PI / 180));         //Calculate the pitch angle
  float roll_acc = asin((float)ax / a_total) * (-1 / (M_PI / 180));          //Calculate the roll angle
  
  //Place the MPU-6050 spirit level and note the values in the following two lines for calibration
  pitch_acc -= 0.0;                                                //Accelerometer calibration value for pitch
  roll_acc -= 0.0;                                                 //Accelerometer calibration value for roll

  if(is_init) {
		                                                 //If the IMU is already started
    gyr_pitch = gyr_pitch * 0.9 + pitch_acc * 0.1;           //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
    gyr_roll = gyr_roll * 0.9 + roll_acc * 0.1;              //Correct the drift of the gyro roll angle with the accelerometer roll angle
	}
	else {
		                                                                //At first start
	  gyr_pitch = pitch_acc;                                         //Set the gyro pitch angle equal to the accelerometer pitch angle 
	  gyr_roll = roll_acc;                                           //Set the gyro roll angle equal to the accelerometer roll angle 
	  is_init = 1;                                              //Set the IMU started flag
	}
	

	//To dampen the pitch and roll angles a complementary filter is used
	copter.orientation.euler.pitch = copter.orientation.euler.pitch * 0.9 + gyr_pitch * 0.1;      //Take 90% of the output pitch value and add 10% of the raw pitch value
	copter.orientation.euler.roll = copter.orientation.euler.roll * 0.9 + gyr_roll * 0.1;         //Take 90% of the output roll value and add 10% of the raw roll value       
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