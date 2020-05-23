#ifndef __BMP280_H
#define __BMP280_H

#include "stm32f1xx_hal.h"

//
//   0: using 32-bit values - lowest precision, less code)
//   1: using 64-bit values - more precision, more code
//   2: using float calculations - best precision, lots of code on MCU without FPU
#define BMP280_CALC_TYPE		1

#if (BMP280_CALC_TYPE < 0) || (BMP280_CALC_TYPE > 2)
#error "Please define correct value for BMP280_CALC_TYPE"
#endif

// Enable functions that operating with floats (calculations and return value)
// all the names ending with 'f'
//   0: not supported
//   1: supported
#define BMP280_FLOAT_FUNCTIONS          0



// BMP280 I2C address

#define BMP280_SDO 0
#if BMP280_SDO
#define BMP280_ADDRESS			0x77 << 1  // Device address when SDO = 1
#else
#define BMP280_ADDRESS			0x76 << 1  // Device address when SDO = 0
#endif  


// BMP280 registers

#define BMP280_CHIP_ID			0x58

#define BMP280_REG_ID			0xD0
#define BMP280_REG_RESET		0xE0
#define BMP280_REG_STATUS		0xF3
#define BMP280_REG_CTRL_MEAS	0xF4
#define BMP280_REG_CONFIG		0xF5
#define BMP280_REG_PRESS_MSB	0xF7
#define BMP280_REG_PRESS_LSB	0xF8
#define BMP280_REG_PRESS_XLSB	0xF9
#define BMP280_REG_TEMP_MSB		0xFA
#define BMP280_REG_TEMP_LSB		0xFB
#define BMP280_REG_TEMP_XLSB	0xFC

#define BMP280_REG_CALIB00		0x88
#define BMP280_REG_CALIB25		0xA1


// BMP280 register bits

// Software reset
#define BMP280_SOFT_RESET_KEY           0xB6 // Value to call a software chip reset routine

// Status register (0xF3)
#define BMP280_STATUS_MSK               0x09 // Mask to clear unused bits
#define BMP280_STATUS_IM_UPDATE         0x01 // Status register bit 0 (NVM data being copied to image registers)
#define BMP280_STATUS_MEASURING         0x08 // Status register bit 3 (conversion is running)

// Pressure and temperature control register (0xF4)
//   Temperature oversampling (osrs_t [7:5])
#define BMP280_OSRS_T_MSK               0xE0 // 'osrs_t' mask
#define BMP280_OSRS_T_SKIP              0x00 // Skipped
#define BMP280_OSRS_T_x1                0x20 // x1
#define BMP280_OSRS_T_x2                0x40 // x2
#define BMP280_OSRS_T_x4                0x60 // x4
#define BMP280_OSRS_T_x8                0x80 // x8
#define BMP280_OSRS_T_x16               0xA0 // x16

//   Pressure oversampling (osrs_p [4:2])
#define BMP280_OSRS_P_MSK               0x1C // 'osrs_p' mask
#define BMP280_OSRS_P_SKIP              0x00 // Skipped
#define BMP280_OSRS_P_x1                0x04 // x1
#define BMP280_OSRS_P_x2                0x08 // x2
#define BMP280_OSRS_P_x4                0x0C // x4
#define BMP280_OSRS_P_x8                0x10 // x8
#define BMP280_OSRS_P_x16               0x14 // x16

//   Power mode of the device (mode [1:0])
#define BMP280_MODE_MSK                 0x03 // 'mode' mask
#define BMP280_MODE_SLEEP               0x00 // Sleep mode
#define BMP280_MODE_FORCED              0x01 // Forced mode
#define BMP280_MODE_FORCED2             0x02 // Forced mode
#define BMP280_MODE_NORMAL              0x03 // Normal mode


// Configuration register: set rate, filter and interface options (0xF5)
//   Inactive duration in normal mode (t_sb [7:5])
#define BMP280_STBY_MSK                 0xE0 // 't_sb' mask
#define BMP280_STBY_0p5ms               0x00 // 0.5ms
#define BMP280_STBY_62p5ms              0x20 // 62.5ms
#define BMP280_STBY_125ms               0x40 // 125ms
#define BMP280_STBY_250ms               0x60 // 250ms
#define BMP280_STBY_500ms               0x80 // 500ms
#define BMP280_STBY_1s                  0xA0 // 1s
#define BMP280_STBY_2s                  0xC0 // 2s
#define BMP280_STBY_4s                  0xE0 // 4s

//   Time constant of the IIR filter (filter [4:2])
#define BMP280_FILTER_MSK               0x1C // 'filter' mask
#define BMP280_FILTER_OFF               0x00 // Off
#define BMP280_FILTER_2                 0x04 // 2
#define BMP280_FILTER_4                 0x08 // 4
#define BMP280_FILTER_8                 0x0C // 8
#define BMP280_FILTER_16                0x10 // 16


// Constant for Pascals to millimeters of mercury conversion
#define BMP280_MMHG_Q0_22               ((uint32_t)31460U) // 0.00750061683 in Q0.22 format


// Definitions of values for absent readings
#define BMP280_NO_TEMPERATURE           ((int32_t)0x80000)
#define BMP280_NO_PRESSURE              ((int32_t)0x80000)


typedef struct {
	uint16_t dig_T1;
	int16_t  dig_T2;
	int16_t  dig_T3;
	uint16_t dig_P1;
	int16_t  dig_P2;
	int16_t  dig_P3;
	int16_t  dig_P4;
	int16_t  dig_P5;
	int16_t  dig_P6;
	int16_t  dig_P7;
	int16_t  dig_P8;
	int16_t  dig_P9;
} BMP280_cal_param_t;

/**
 * Mode of BMP280 module operation.
 * Forced - Measurement is initiated by user.
 * Normal - Continues measurement.
 */
typedef enum {
	BMP280_SLEEP_MODE  = BMP280_MODE_SLEEP,
	BMP280_FORCED_MODE = BMP280_MODE_FORCED,
	BMP280_NORMAL_MODE = BMP280_MODE_NORMAL
} BMP280_mode;

typedef enum {
	BMP280_FILTER_COEF_OFF = BMP280_FILTER_OFF,
	BMP280_FILTER_COEF_2   = BMP280_FILTER_2,
	BMP280_FILTER_COEF_4   = BMP280_FILTER_4,
	BMP280_FILTER_COEF_8   = BMP280_FILTER_8,
	BMP280_FILTER_COEF_16  = BMP280_FILTER_16
} BMP280_filter;

/**
 * Temperature oversampling settings
 */
typedef enum {
	BMP280_TEMP_SKIPPED			= BMP280_OSRS_T_SKIP,
	BMP280_TEMP_ULTRA_LOW_POWER = BMP280_OSRS_T_x1,
	BMP280_TEMP_LOW_POWER		= BMP280_OSRS_T_x2,
	BMP280_TEMP_STANDARD		= BMP280_OSRS_T_x4,
	BMP280_TEMP_HIGH_RES		= BMP280_OSRS_T_x8,
	BMP280_TEMP_ULTRA_HIGH_RES	= BMP280_OSRS_T_x16
} BMP280_oversampling_temp;

/**
 * Pressure oversampling settings
 */
typedef enum {
	BMP280_PRESS_SKIPPED         = BMP280_OSRS_P_SKIP,
	BMP280_PRESS_ULTRA_LOW_POWER = BMP280_OSRS_P_x1,
	BMP280_PRESS_LOW_POWER       = BMP280_OSRS_P_x2,
	BMP280_PRESS_STANDARD        = BMP280_OSRS_P_x4,
	BMP280_PRESS_HIGH_RES        = BMP280_OSRS_P_x8,
	BMP280_PRESS_ULTRA_HIGH_RES  = BMP280_OSRS_P_x16
} BMP280_oversampling_press;

/**
 * Stand by time between measurements in normal mode
 */
typedef enum {
	BMP280_STANDBY_05_MS		= BMP280_STBY_0p5ms,
	BMP280_STANDBY_62_MS		= BMP280_STBY_62p5ms,
	BMP280_STANDBY_125_MS		= BMP280_STBY_125ms,
	BMP280_STANDBY_250_MS		= BMP280_STBY_250ms,
	BMP280_STANDBY_500_MS		= BMP280_STBY_500ms,
	BMP280_STANDBY_1000_MS		= BMP280_STBY_1s,
	BMP280_STANDBY_2000_MS		= BMP280_STBY_2s,
	BMP280_STANDBY_4000_MS		= BMP280_STBY_4s,
} BMP280_standby_time;


typedef struct
{
	
} BMP280_sensor_t;


typedef struct
{
	I2C_HandleTypeDef*			hi2c;
	BMP280_mode					mode;
	BMP280_filter				filter_coeff;
	BMP280_oversampling_temp	ost;
	BMP280_oversampling_press	osp;
	BMP280_standby_time			stby_time;
} BMP280_init_t;

typedef struct
{
	I2C_HandleTypeDef*			hi2c;
	BMP280_cal_param_t			calibration_data;
	uint32_t					pressure;
	int32_t						temperature;
} BMP280_t;

BMP280_t bmp280;

HAL_StatusTypeDef BMP280_init(BMP280_init_t* bmp280_init);

HAL_StatusTypeDef BMP280_who_am_i();

HAL_StatusTypeDef BMP280_reset();

HAL_StatusTypeDef BMP280_set_mode(BMP280_mode mode);

HAL_StatusTypeDef BMP280_set_filter(BMP280_filter filter);

HAL_StatusTypeDef BMP280_set_standby_time(BMP280_standby_time tsb);

HAL_StatusTypeDef BMP280_set_temp_oversampling(BMP280_oversampling_temp osrs);

HAL_StatusTypeDef BMP280_set_press_oversampling(uint8_t osrs);

HAL_StatusTypeDef BMP280_read_calibration();

HAL_StatusTypeDef BMP280_read_temp_raw(int32_t *temp);

HAL_StatusTypeDef BMP280_read_press_raw(int32_t *press);

HAL_StatusTypeDef BMP280_read_press_temp_raw(int32_t *press, int32_t *temp);

int32_t BMP280_get_temp_celsius_x_100(int32_t temp);

uint32_t BMP280_get_pressure_mPa(int32_t press);

float BMP280_get_altitude(int32_t press);


#endif /*__BMP280_H */