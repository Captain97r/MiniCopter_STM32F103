#ifndef __MPU9250_H
#define __MPU9250_H

#include "stm32f1xx_hal.h"


//Magnetometer Registers
#define AK8963_ADDRESS		0x0C<<1
#define AK8963_WHO_AM_I		0x00 // should return 0x48
#define AK8963_INFO			0x01
#define AK8963_ST1			0x02  // data ready status bit 0
#define AK8963_XOUT_L		0x03  // data
#define AK8963_XOUT_H		0x04
#define AK8963_YOUT_L		0x05
#define AK8963_YOUT_H		0x06
#define AK8963_ZOUT_L		0x07
#define AK8963_ZOUT_H		0x08
#define AK8963_ST2			0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL			0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC			0x0C  // Self test control
#define AK8963_I2CDIS		0x0F  // I2C disable
#define AK8963_ASAX			0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY			0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ			0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO	0x00                  
#define SELF_TEST_Y_GYRO	0x01                                                                          
#define SELF_TEST_Z_GYRO	0x02

/*#define X_FINE_GAIN			0x03 // [7:0] fine gain
#define Y_FINE_GAIN			0x04
#define Z_FINE_GAIN			0x05
#define XA_OFFSET_H			0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC		0x07
#define YA_OFFSET_H			0x08
#define YA_OFFSET_L_TC		0x09
#define ZA_OFFSET_H			0x0A
#define ZA_OFFSET_L_TC		0x0B */

#define SELF_TEST_X_ACCEL	0x0D
#define SELF_TEST_Y_ACCEL	0x0E    
#define SELF_TEST_Z_ACCEL	0x0F

#define SELF_TEST_A			0x10

#define XG_OFFSET_H			0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L			0x14
#define YG_OFFSET_H			0x15
#define YG_OFFSET_L			0x16
#define ZG_OFFSET_H			0x17
#define ZG_OFFSET_L			0x18
#define SMPLRT_DIV			0x19
#define CONFIG				0x1A
#define GYRO_CONFIG			0x1B
#define ACCEL_CONFIG		0x1C
#define ACCEL_CONFIG2		0x1D
#define LP_ACCEL_ODR		0x1E   
#define WOM_THR				0x1F   

#define MOT_DUR				0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR			0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR			0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN				0x23
#define I2C_MST_CTRL		0x24   
#define I2C_SLV0_ADDR		0x25
#define I2C_SLV0_REG		0x26
#define I2C_SLV0_CTRL		0x27
#define I2C_SLV1_ADDR		0x28
#define I2C_SLV1_REG		0x29
#define I2C_SLV1_CTRL		0x2A
#define I2C_SLV2_ADDR		0x2B
#define I2C_SLV2_REG		0x2C
#define I2C_SLV2_CTRL		0x2D
#define I2C_SLV3_ADDR		0x2E
#define I2C_SLV3_REG		0x2F
#define I2C_SLV3_CTRL		0x30
#define I2C_SLV4_ADDR		0x31
#define I2C_SLV4_REG		0x32
#define I2C_SLV4_DO			0x33
#define I2C_SLV4_CTRL		0x34
#define I2C_SLV4_DI			0x35
#define I2C_MST_STATUS		0x36
#define INT_PIN_CFG			0x37
#define INT_ENABLE			0x38
#define DMP_INT_STATUS		0x39  // Check DMP interrupt
#define INT_STATUS			0x3A
#define ACCEL_XOUT_H		0x3B
#define ACCEL_XOUT_L		0x3C
#define ACCEL_YOUT_H		0x3D
#define ACCEL_YOUT_L		0x3E
#define ACCEL_ZOUT_H		0x3F
#define ACCEL_ZOUT_L		0x40
#define TEMP_OUT_H			0x41
#define TEMP_OUT_L			0x42
#define GYRO_XOUT_H			0x43
#define GYRO_XOUT_L			0x44
#define GYRO_YOUT_H			0x45
#define GYRO_YOUT_L			0x46
#define GYRO_ZOUT_H			0x47
#define GYRO_ZOUT_L			0x48
#define EXT_SENS_DATA_00	0x49
#define EXT_SENS_DATA_01	0x4A
#define EXT_SENS_DATA_02	0x4B
#define EXT_SENS_DATA_03	0x4C
#define EXT_SENS_DATA_04	0x4D
#define EXT_SENS_DATA_05	0x4E
#define EXT_SENS_DATA_06	0x4F
#define EXT_SENS_DATA_07	0x50
#define EXT_SENS_DATA_08	0x51
#define EXT_SENS_DATA_09	0x52
#define EXT_SENS_DATA_10	0x53
#define EXT_SENS_DATA_11	0x54
#define EXT_SENS_DATA_12	0x55
#define EXT_SENS_DATA_13	0x56
#define EXT_SENS_DATA_14	0x57
#define EXT_SENS_DATA_15	0x58
#define EXT_SENS_DATA_16	0x59
#define EXT_SENS_DATA_17	0x5A
#define EXT_SENS_DATA_18	0x5B
#define EXT_SENS_DATA_19	0x5C
#define EXT_SENS_DATA_20	0x5D
#define EXT_SENS_DATA_21	0x5E
#define EXT_SENS_DATA_22	0x5F
#define EXT_SENS_DATA_23	0x60
#define MOT_DETECT_STATUS	0x61
#define I2C_SLV0_DO			0x63
#define I2C_SLV1_DO			0x64
#define I2C_SLV2_DO			0x65
#define I2C_SLV3_DO			0x66
#define I2C_MST_DELAY_CTRL	0x67
#define SIGNAL_PATH_RESET	0x68
#define MOT_DETECT_CTRL		0x69
#define USER_CTRL			0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1			0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2			0x6C
#define DMP_BANK			0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT			0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG				0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1			0x70
#define DMP_REG_2			0x71 
#define FIFO_COUNTH			0x72
#define FIFO_COUNTL			0x73
#define FIFO_R_W			0x74
#define WHO_AM_I_MPU9250	0x75 // Should return 0x71
#define XA_OFFSET_H			0x77
#define XA_OFFSET_L			0x78
#define YA_OFFSET_H			0x7A
#define YA_OFFSET_L			0x7B
#define ZA_OFFSET_H			0x7D
#define ZA_OFFSET_L			0x7E

// Using the MSENSR-9250 breakout board, ADO is set to 0 
// Seven-bit device address is 110100 for ADO = 0 and 110101 for ADO = 1
//mbed uses the eight-bit device address, so shift seven-bit addresses left by one!
#define ADO 0
#if ADO
#define MPU9250_ADDRESS 0x69<<1  // Device address when ADO = 1
#else
#define MPU9250_ADDRESS 0x68<<1  // Device address when ADO = 0
#endif  


// AK8963 Register bits definitions
#define AK8963_WHO_AM_I_RSP 0x48

// CNTL Register
#define AK8963_POWER_DOWN	0x00
#define AK8963_SINGLE		0x01
#define AK8963_8HZ			0x02
#define AK8963_100HZ		0x06
#define AK8963_SELF_TEST	0x08
#define AK8963_FUSE_ROM		0x0F

#define AK8963_SCALE_14BIT	0x00 << 4
#define AK8963_SCALE_16BIT	0x01 << 4

// ASTC Register
#define AK8963_ASTC_SELF	0x40

// ST1 Register
#define AK8963_DRDY			0x01
#define AK8963_DOR			0x02

// ST2 Register
#define AK8963_HOFL			0x08


// MPU9250 Register bits definitions
#define MPU9250_WHO_AM_I_RSP 0x71


// MPU9250 Power Management 1 Register
#define MPU9250_CLK_INTRNL	0x00
#define MPU9250_CLK_PLL		0x01
#define MPU9250_CLK_STOP	0x07

#define MPU9250_POWER_DOWN	0x08
#define MPU9250_GYRO_STBY	0x10
#define MPU9250_CYCLE		0x20
#define MPU9250_SLEEP		0x40
#define MPU9250_HARD_RESET	0x80


// MPU9250 Power Management 2 Register
#define MPU9250_ENABLE_ALL	0x00
#define MPU9250_DISABLE_ZG	0x01
#define MPU9250_DISABLE_YG	0x02
#define MPU9250_DISABLE_XG	0x04
#define MPU9250_DISABLE_ZA	0x08
#define MPU9250_DISABLE_YA	0x10
#define MPU9250_DISABLE_XA	0x20


// MPU9250 Configuration Register
#define MPU9250_DLPF_CFG_0	0x00
#define MPU9250_DLPF_CFG_1	0x01
#define MPU9250_DLPF_CFG_2	0x02
#define MPU9250_DLPF_CFG_3	0x03
#define MPU9250_DLPF_CFG_4	0x04
#define MPU9250_DLPF_CFG_5	0x05
#define MPU9250_DLPF_CFG_6	0x06
#define MPU9250_DLPF_CFG_7	0x07

#define MPU9250_FSYNC_DIS	0x00 << 3
#define MPU9250_FSYNC_TEMP	0x01 << 3
#define MPU9250_FSYNC_XG	0x02 << 3
#define MPU9250_FSYNC_YG	0x03 << 3
#define MPU9250_FSYNC_ZG	0x04 << 3
#define MPU9250_FSYNC_XA	0x05 << 3
#define MPU9250_FSYNC_YA	0x06 << 3
#define MPU9250_FSYNC_ZA	0x07 << 3

#define MPU9250_FIFO_RPLC	0x40


// MPU9250 Gyroscope Configuration Register
#define GYRO_FCHOISE_2b00	0x03
#define GYRO_FCHOISE_2b01	0x02
#define GYRO_FCHOISE_2b10	0x01
#define GYRO_FCHOISE_2b11	0x00

#define GYRO_FS_250DPS		0x00 << 3
#define GYRO_FS_500DPS		0x01 << 3
#define GYRO_FS_1000DPS		0x02 << 3
#define GYRO_FS_2000DPS		0x03 << 3

#define GYRO_SELF_TEST_X	0x80
#define GYRO_SELF_TEST_Y	0x40
#define GYRO_SELF_TEST_Z	0x20


// MPU9250 Accelerometer Configuration 1 Register
#define ACCEL_FS_2G			0x00 << 3
#define ACCEL_FS_4G			0x01 << 3
#define ACCEL_FS_8G			0x02 << 3
#define ACCEL_FS_16G		0x03 << 3

#define ACCEL_SELF_TEST_X	0x80
#define ACCEL_SELF_TEST_Y	0x40
#define ACCEL_SELF_TEST_Z	0x20


// MPU9250 Accelerometer Configuration 2 Register
#define	ACCEL_DLPF_CFG_0	0x00
#define ACCEL_DLPF_CFG_1	0x01
#define ACCEL_DLPF_CFG_2	0x02
#define ACCEL_DLPF_CFG_3	0x03
#define ACCEL_DLPF_CFG_4	0x04
#define ACCEL_DLPF_CFG_5	0x05
#define ACCEL_DLPF_CFG_6	0x06
#define ACCEL_DLPF_CFG_7	0x07

#define ACCEL_FCHOISE_0		0x01 << 3
#define ACCEL_FCHOISE_1		0x00 << 3


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
} sensor_t;

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
	sensor_t					accelerometer;
	sensor_t					gyroscope;
	sensor_t					magnetometer;
	float						temperature;
} MPU9250_t;


MPU9250_t mpu9250;

HAL_StatusTypeDef MPU9250_who_am_i();
HAL_StatusTypeDef MPU9250_init(MPU9250_init_t *mpu9250_init);

void MPU9250_calibrate();
void AK8963_calibrate();

HAL_StatusTypeDef MPU9250_self_test();
HAL_StatusTypeDef AK8963_self_test();

void MPU9250_get_accel();

void MPU9250_get_gyro();

void MPU9250_get_mag();

void MPU9250_get_temp();

void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
void toEuler();

#endif /*__MPU9250_H */
