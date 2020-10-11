#ifndef __MPU9250_H
#define __MPU9250_H

#include "stm32f1xx_hal.h"
#include "MPU9250_map.h"
#include "matrixmath.h"

#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69<<1  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68<<1  // Device address when ADO = 0
#endif  


typedef enum
{
	GYRO_FREQ_DEFAULT = 0,
	GYRO_FREQ_32KHZ,
	GYRO_FREQ_8KHZ,
	GYRO_FREQ_1KHZ,
} GYRO_FREQUENCY;

typedef enum
{
	GYRO_BANDWIDTH_DEFAULT = 0,
	GYRO_BANDWIDTH_8800HZ,
	GYRO_BANDWIDTH_3600HZ,
	GYRO_BANDWIDTH_250HZ,
	GYRO_BANDWIDTH_184HZ,
	GYRO_BANDWIDTH_92HZ,
	GYRO_BANDWIDTH_41HZ,
	GYRO_BANDWIDTH_20HZ,
	GYRO_BANDWIDTH_10HZ,
	GYRO_BANDWIDTH_5HZ
} GYRO_BANDWIDTH;

typedef enum
{
	ACCEL_FREQ_DEFAULT = 0,
	ACCEL_FREQ_4KHZ,
	ACCEL_FREQ_1KHZ,
} ACCEL_FREQUENCY;

typedef enum
{
	ACCEL_BANDWIDTH_DEFAULT = 0,
	ACCEL_BANDWIDTH_1046HZ,
	ACCEL_BANDWIDTH_420HZ,
	ACCEL_BANDWIDTH_218HZ,
	ACCEL_BANDWIDTH_99HZ,
	ACCEL_BANDWIDTH_44HZ,
	ACCEL_BANDWIDTH_21HZ,
	ACCEL_BANDWIDTH_10HZ,
	ACCEL_BANDWIDTH_5HZ,
} ACCEL_BANDWIDTH;

typedef enum {
	AFS_2G	= ACCEL_FS_2G,
	AFS_4G	= ACCEL_FS_4G,
	AFS_8G	= ACCEL_FS_8G,
	AFS_16G = ACCEL_FS_16G
} ACCEL_SCALE;

typedef enum {
	GFS_250DPS	= GYRO_FS_250DPS,
	GFS_500DPS	= GYRO_FS_500DPS,
	GFS_1000DPS = GYRO_FS_1000DPS,
	GFS_2000DPS = GYRO_FS_2000DPS
} GYRO_SCALE;

typedef enum {
	MFS_14BITS = AK8963_SCALE_14BIT,
	MFS_16BITS = AK8963_SCALE_16BIT
} MAG_SCALE;

typedef enum
{
	MAG_FREQ_8HZ = AK8963_8HZ,
	MAG_FREQ_100HZ = AK8963_100HZ
} MAG_FREQUENCY;

typedef struct
{
	float x;
	float y;
	float z;
} sensor_data_t;

typedef struct
{
	sensor_data_t				data;
	sensor_data_t				offset;
	sensor_data_t				calibration;
	float						resolution;
	Dcmf						dcm;
} MPU9250_sensor_t;

typedef struct
{
	I2C_HandleTypeDef*			hi2c;
	ACCEL_SCALE					accel_scale;
	ACCEL_FREQUENCY				accel_freq;
	ACCEL_BANDWIDTH				accel_bandwidth;
	GYRO_SCALE					gyro_scale;
	GYRO_FREQUENCY				gyro_freq;
	GYRO_BANDWIDTH				gyro_bandwidth;
	MAG_SCALE					mag_scale;
	MAG_FREQUENCY				mag_freq;
} MPU9250_init_t;

typedef struct
{
	I2C_HandleTypeDef*			hi2c;
	HAL_StatusTypeDef			isPresent;
	MPU9250_sensor_t			accelerometer;
	MPU9250_sensor_t			gyroscope;
	MPU9250_sensor_t			magnetometer;
	float						temperature;
} MPU9250_t;

MPU9250_t mpu9250;


HAL_StatusTypeDef MPU9250_who_am_i();
HAL_StatusTypeDef MPU9250_init(MPU9250_init_t *mpu9250_init);
HAL_StatusTypeDef MPU9250_self_test();
void MPU9250_calibrate();
void MPU9250_get_accel();
void MPU9250_get_gyro();
void MPU9250_get_mag();
void MPU9250_get_temp();

HAL_StatusTypeDef AK8963_who_am_i();
HAL_StatusTypeDef AK8963_self_test();
void AK8963_calibrate();

void save_calibration_to_flash();
void load_calibration_from_flash();

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void toEuler();

#endif /*__MPU9250_H */
