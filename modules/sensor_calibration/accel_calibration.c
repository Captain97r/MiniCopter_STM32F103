#include "accel_calibration.h"
#include "MPU9250.h"


void accelerometer_calibration()
{
	
	// Set correction values to default
	for(uint8_t i = 0 ; i < 3 ; i++)
		for(uint8_t j = 0 ; j < 3 ; j++)
			mpu9250.accelerometer.dcm[i][j] = 0;
	
	mpu9250.accelerometer.dcm[0][0] = 1;
	mpu9250.accelerometer.dcm[1][1] = 1;
	mpu9250.accelerometer.dcm[2][2] = 1;
	
	mpu9250.accelerometer.offset.x = 0;
	mpu9250.accelerometer.offset.y = 0;
	mpu9250.accelerometer.offset.z = 0;
	
	float accel_corr_ref[6][3] = {  { G_CONST,      0,          0       },          // nose down
                                    {-G_CONST,      0,          0       },          // nose up
                                    { 0,            G_CONST,    0       },          // left side down
                                    { 0,           -G_CONST,    0       },          // right side down
                                    { 0,            0,          G_CONST },          // level
                                    { 0,            0,         -G_CONST }		    // on back
	};
	
	float accel_raw_ref[6][3];
	for (uint8_t i = 0; i < 6; i++)
	{
		float data[3] = { 0, 0, 0 };
		for (uint8_t n = 0; n < 100; n++)
		{
			MPU9250_get_accel();
			data[0] += mpu9250.accelerometer.data.x;
			data[1] += mpu9250.accelerometer.data.y;
			data[2] += mpu9250.accelerometer.data.z;
			HAL_Delay(5);
		}
		
		data[0] /= 100;
		data[1] /= 100;
		data[2] /= 100;
		
		accel_raw_ref[i][0] =  data[0];
		accel_raw_ref[i][1] =  data[1];
		accel_raw_ref[i][2] =  data[2];
	}
	
	mpu9250.accelerometer.offset.x = (accel_raw_ref[0][0] + accel_raw_ref[1][0]) / 2;
	mpu9250.accelerometer.offset.y = (accel_raw_ref[2][1] + accel_raw_ref[3][1]) / 2;
	mpu9250.accelerometer.offset.z = (accel_raw_ref[4][2] + accel_raw_ref[5][2]) / 2;
	
	float accel_offs[3] = { mpu9250.accelerometer.offset.x, mpu9250.accelerometer.offset.y, mpu9250.accelerometer.offset.z };
	
	float a[3][3];
		for (uint8_t i = 0; i < 3; i++)
			for (uint8_t j = 0; j < 3; j++)
				a[i][j] = accel_raw_ref[i * 2][j] - accel_offs[j];
	
	// TODO: implement an inverse matrix function and do a^-1 * b
	// Determinant
	float det_a = (a[0][0]*a[1][1]*a[2][2] + a[0][1]*a[1][2]*a[2][0] + a[1][0]*a[2][1]*a[0][2]) - (a[0][2]*a[1][1]*a[2][0] + a[0][1]*a[1][0]*a[2][2] + a[2][1]*a[1][2]*a[0][0]);
		
	// Algebraic additions
	float m[3][3];
	m[0][0] = a[1][1] * a[2][2] - a[1][2] * a[2][1];
	m[0][1] = a[1][2] * a[2][0] - a[1][0] * a[2][2];
	m[0][2] = a[1][0] * a[2][1] - a[1][1] * a[2][0];
	m[1][0] = a[0][2] * a[2][1] - a[0][1] * a[2][2];
	m[1][1] = a[0][0] * a[2][2] - a[0][2] * a[2][0];
	m[1][2] = a[0][1] * a[2][0] - a[0][0] * a[2][1];
	m[2][0] = a[0][1] * a[1][2] - a[0][2] * a[1][1];
	m[2][1] = a[0][2] * a[1][0] - a[0][0] * a[1][2];
	m[2][2] = a[0][0] * a[1][1] - a[0][1] * a[1][0];
	
	float m_t[3][3];
	for (uint8_t i = 0; i < 3; i++)
		for (uint8_t j = 0; j < 3; j++)
			m_t[i][j] = m[j][i];
	
	float inv_a[3][3];
	for (uint8_t i = 0; i < 3; i++)
		for (uint8_t j = 0; j < 3; j++)
			inv_a[i][j] = m_t[i][j] / det_a;
	
	for (uint8_t i = 0; i < 3; i++)
		for (uint8_t j = 0; j < 3; j++)
			mpu9250.accelerometer.dcm[i][j] = inv_a[j][i] * G_CONST;
	
}