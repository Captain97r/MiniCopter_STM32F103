#include "BMP280.h"

#include "math.h"


#if (BMP280_CALC_TYPE == 2) || (BMP280_FLOAT_FUNCTIONS)
static float t_fine_f;
#endif // (BMP280_CALC_TYPE == 2) || (BMP280_FLOAT_FUNCTIONS)
#if (BMP280_CALC_TYPE == 0) || (BMP280_CALC_TYPE == 1)
static int32_t t_fine;
#endif // (BMP280_CALC_TYPE == 0) || (BMP280_CALC_TYPE == 1)

static int32_t init_press = 0;

HAL_StatusTypeDef BMP280_read(uint8_t *data)
{
	return HAL_I2C_Master_Receive(bmp280.hi2c, (uint16_t)BMP280_ADDRESS, (uint8_t *)data, 1, 1000);
}


HAL_StatusTypeDef BMP280_write(uint8_t *data, uint8_t size)
{
	return HAL_I2C_Master_Transmit(bmp280.hi2c, (uint16_t)BMP280_ADDRESS, (uint8_t *)data, size, 1000);
}

HAL_StatusTypeDef BMP280_read_reg(uint8_t address, uint8_t *data)
{
	uint8_t addr[1] = { address };
	HAL_StatusTypeDef status = BMP280_write(addr, 1);
	
	if (status != HAL_OK)
		return HAL_ERROR;
	
	return BMP280_read(data);
}

HAL_StatusTypeDef BMP280_burst_read_reg(uint8_t address, uint8_t *data, uint8_t size)
{
	uint8_t addr[1] = { address };
	HAL_StatusTypeDef status = BMP280_write(addr, 1);
	
	if (status != HAL_OK)
		return HAL_ERROR;
	
	return HAL_I2C_Master_Receive(bmp280.hi2c, (uint16_t)BMP280_ADDRESS, (uint8_t *)data, size, 1000);
}

HAL_StatusTypeDef BMP280_write_reg(uint8_t address, uint8_t data)
{
	uint8_t addr[2] = { address, data };
	return BMP280_write(addr, 2);
}

HAL_StatusTypeDef BMP280_set_init_pressure()
{
	int32_t temp_raw, press_raw;
	if (BMP280_read_press_temp_raw(&press_raw, &temp_raw) == HAL_ERROR)
		return HAL_ERROR;
	
	BMP280_get_temp_celsius_x_100(temp_raw);
	init_press = BMP280_get_pressure_mPa(press_raw);
	
	return HAL_OK;
}




HAL_StatusTypeDef BMP280_init(BMP280_init_t* bmp280_init)
{
	bmp280.hi2c = bmp280_init->hi2c;
	
	BMP280_reset();
	HAL_Delay(100);
	
	if (BMP280_read_calibration() == HAL_ERROR)
		return HAL_ERROR;
	
	if (BMP280_set_filter(bmp280_init->filter_coeff) == HAL_ERROR)
		return HAL_ERROR;
	
	if (BMP280_set_standby_time(bmp280_init->stby_time) == HAL_ERROR)
		return HAL_ERROR;
	
	if (BMP280_set_temp_oversampling(bmp280_init->ost) == HAL_ERROR)
		return HAL_ERROR;
	
	if (BMP280_set_press_oversampling(bmp280_init->osp) == HAL_ERROR)
		return HAL_ERROR;
	
	if (BMP280_set_mode(bmp280_init->mode) == HAL_ERROR)
		return HAL_ERROR;
	
	HAL_Delay(50);
	if (BMP280_set_init_pressure() == HAL_ERROR)
		return HAL_ERROR;
	
	return HAL_OK;
}


HAL_StatusTypeDef BMP280_who_am_i()
{
	uint8_t responce;
	BMP280_read_reg(BMP280_REG_ID, &responce);
	return (responce == BMP280_CHIP_ID) ? HAL_OK : HAL_ERROR;
}


HAL_StatusTypeDef BMP280_reset() 
{
	return BMP280_write_reg(BMP280_REG_RESET, BMP280_SOFT_RESET_KEY);
}

HAL_StatusTypeDef BMP280_set_mode(BMP280_mode mode) 
{
	uint8_t responce;

	if (BMP280_read_reg(BMP280_REG_CTRL_MEAS, &responce))
		return HAL_ERROR;
	
	responce &= ~BMP280_MODE_MSK;
	responce |= mode & BMP280_MODE_MSK;
	return BMP280_write_reg(BMP280_REG_CTRL_MEAS, responce);
}

HAL_StatusTypeDef BMP280_set_filter(BMP280_filter filter) 
{
	uint8_t responce;

	if (BMP280_read_reg(BMP280_REG_CONFIG, &responce))
		return HAL_ERROR;
		
	responce &= ~BMP280_FILTER_MSK;
	responce |= filter & BMP280_FILTER_MSK;
	return BMP280_write_reg(BMP280_REG_CONFIG, responce);
}

HAL_StatusTypeDef BMP280_set_standby_time(BMP280_standby_time tsb) 
{
	uint8_t responce;

	if (BMP280_read_reg(BMP280_REG_CONFIG, &responce))
		return HAL_ERROR;
	
	responce &= ~BMP280_STBY_MSK;
	responce |= tsb & BMP280_STBY_MSK;
	return BMP280_write_reg(BMP280_REG_CONFIG, responce);
}

HAL_StatusTypeDef BMP280_set_temp_oversampling(BMP280_oversampling_temp osrs)
{
	uint8_t responce;

	if (BMP280_read_reg(BMP280_REG_CTRL_MEAS, &responce))
		return HAL_ERROR;
	
	responce &= ~BMP280_OSRS_T_MSK;
	responce |= osrs & BMP280_OSRS_T_MSK;
	return BMP280_write_reg(BMP280_REG_CTRL_MEAS, responce);
}

HAL_StatusTypeDef BMP280_set_press_oversampling(uint8_t osrs) 
{
	uint8_t responce;

	if (BMP280_read_reg(BMP280_REG_CTRL_MEAS, &responce))
		return HAL_ERROR;
	
	responce &= ~BMP280_OSRS_P_MSK;
	responce |= osrs & BMP280_OSRS_P_MSK;
	return BMP280_write_reg(BMP280_REG_CTRL_MEAS, responce);
}

HAL_StatusTypeDef BMP280_read_calibration() 
{
	return BMP280_burst_read_reg(BMP280_REG_CALIB00, (uint8_t *)(&bmp280.calibration_data), 24);
}

HAL_StatusTypeDef BMP280_read_temp_raw(int32_t *temp) 
{
	uint8_t buf[3];

	// Default result value
	*temp = BMP280_NO_TEMPERATURE;

	if (BMP280_burst_read_reg(BMP280_REG_TEMP_MSB, (uint8_t *)buf, 3) == HAL_OK) {
		*temp = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
		return HAL_OK;
	}

	return HAL_ERROR;
}


HAL_StatusTypeDef BMP280_read_press_raw(int32_t *press) 
{
	uint8_t buf[3];

	// Default result value
	*press = BMP280_NO_PRESSURE;

	if (BMP280_burst_read_reg(BMP280_REG_PRESS_MSB, (uint8_t *)buf, 3) == HAL_OK) 
	{
		*press = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
		return HAL_OK;
	}

	return HAL_ERROR;
}


HAL_StatusTypeDef BMP280_read_press_temp_raw(int32_t *press, int32_t *temp) 
{
	uint8_t buf[6];

	// Default result value
	*press = BMP280_NO_PRESSURE;
	*temp = BMP280_NO_TEMPERATURE;

	if (BMP280_burst_read_reg(BMP280_REG_PRESS_MSB, (uint8_t *)buf, 6) == HAL_OK) 
	{
		*press = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
		*temp = (int32_t)((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4));
		return HAL_OK;
	}

	return HAL_ERROR;
}


int32_t BMP280_get_temp_celsius_x_100(int32_t temp) 
{
#if (BMP280_CALC_TYPE != 2)
	// Integer calculations

	t_fine  = ((((temp >> 3) - ((int32_t)bmp280.calibration_data.dig_T1 << 1))) \
			* ((int32_t)bmp280.calibration_data.dig_T2)) >> 11;
	t_fine += (((((temp >> 4) - ((int32_t)bmp280.calibration_data.dig_T1)) \
			* ((temp >> 4) - ((int32_t)bmp280.calibration_data.dig_T1))) >> 12) \
			* ((int32_t)bmp280.calibration_data.dig_T3)) >> 14;

	return ((t_fine * 5) + 128) >> 8;
#else
	// Float calculations

	float v_x1, v_x2;

	v_x1 = (((float)UT) / 16384.0F - ((float)cal_param.dig_T1) / 1024.0F) * \
			((float)cal_param.dig_T2);
	v_x2 = ((float)UT) / 131072.0F - ((float)cal_param.dig_T1) / 8192.0F;
	v_x2 = (v_x2 * v_x2) * ((float)cal_param.dig_T3);
	t_fine_f = v_x1 + v_x2;

	return (int32_t)(((v_x1 + v_x2) / 5120.0F) * 100.0F);
#endif // BMP280_CALC_TYPE
}


uint32_t BMP280_get_pressure_mPa(int32_t press) 
{
#if (BMP280_CALC_TYPE == 0)
	// 32-bit only calculations
	int32_t v1, v2;
	uint32_t p;

	v1 = (((int32_t)t_fine) >> 1) - (int32_t)64000;
	v2 = (((v1 >> 2) * (v1 >> 2)) >> 11) * ((int32_t)bmp280.calibration_data.dig_P6);
	v2 = v2 + ((v1 * ((int32_t)bmp280.calibration_data.dig_P5)) << 1);
	v2 = (v2 >> 2) + (((int32_t)bmp280.calibration_data.dig_P4) << 16);
	v1 = (((bmp280.calibration_data.dig_P3 * (((v1 >> 2) * (v1 >> 2)) >> 13)) >> 3) + \
			((((int32_t)bmp280.calibration_data.dig_P2) * v1) >> 1)) >> 18;
	v1 = (((32768 + v1)) * ((int32_t)bmp280.calibration_data.dig_P1)) >> 15;
	if (v1 == 0) {
		// avoid exception caused by division by zero
		return 0;
	}
	p = (((uint32_t)(((int32_t)1048576) - press) - (v2 >> 12))) * 3125;
	if (p < 0x80000000) {
		p = (p << 1) / ((uint32_t)v1);
	}
	else {
		p = (p / (uint32_t)v1) << 1;
	}
	v1 = (((int32_t)bmp280.calibration_data.dig_P9) * \
			((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
	v2 = (((int32_t)(p >> 2)) * ((int32_t)bmp280.calibration_data.dig_P8)) >> 13;
	p = (uint32_t)((int32_t)p + ((v1 + v2 + bmp280.calibration_data.dig_P7) >> 4));

	return p * 1000;
#elif (BMP280_CALC_TYPE == 1)
	// 64-bit calculations
	int64_t v1, v2, p;

	v1 = (int64_t)t_fine - 128000;
	v2 = v1 * v1 * (int64_t)bmp280.calibration_data.dig_P6;
	v2 = v2 + ((v1 * (int64_t)bmp280.calibration_data.dig_P5) << 17);
	v2 = v2 + ((int64_t)bmp280.calibration_data.dig_P4 << 35);
	v1 = ((v1 * v1 * (int64_t)bmp280.calibration_data.dig_P3) >> 8) + \
			((v1 * (int64_t)bmp280.calibration_data.dig_P2) << 12);
	v1 = (((((int64_t)1) << 47) + v1)) * ((int64_t)bmp280.calibration_data.dig_P1) >> 33;
	if (v1 == 0) {
		// avoid exception caused by division by zero
		return 0;
	}
	p = 1048576 - press;
	p = (((p << 31) - v2) * 3125) / v1;
	v1 = (((int64_t)bmp280.calibration_data.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	v2 = (((int64_t)bmp280.calibration_data.dig_P8) * p) >> 19;
	p = ((p + v1 + v2) >> 8) + ((int64_t)bmp280.calibration_data.dig_P7 << 4);

	return (uint32_t)((p * 1000) >> 8);
#else // BMP280_CALC_TYPE == 2
	// Float calculations
	float v_x1, v_x2, p_f;

	v_x1 = (t_fine_f / 2.0F) - 64000.0F;
	v_x2 = v_x1 * v_x1 * ((float)bmp280.calibration_data.dig_P6) / 32768.0F;
	v_x2 = v_x2 + v_x1 * ((float)bmp280.calibration_data.dig_P5) * 2.0F;
	v_x2 = (v_x2 / 4.0F) + (((float)bmp280.calibration_data.dig_P4) * 65536.0F);
	v_x1 = (((float)bmp280.calibration_data.dig_P3) * v_x1 * v_x1 / 524288.0F + \
			((float)bmp280.calibration_data.dig_P2) * v_x1) / 524288.0F;
	v_x1 = (1.0F + v_x1 / 32768.0F) * ((float)bmp280.calibration_data.dig_P1);
	p_f = 1048576.0F - (float)press;
	if (v_x1 == 0.0F) {
		// Avoid exception caused by division by zero
		return 0;
	}
	p_f = (p_f - (v_x2 / 4096.0F)) * 6250.0F / v_x1;
	v_x1 = ((float)bmp280.calibration_data.dig_P9) * p_f * p_f / 2147483648.0F;
	v_x2 = p_f * ((float)bmp280.calibration_data.dig_P8) / 32768.0F;
	p_f += (v_x1 + v_x2 + ((float)bmp280.calibration_data.dig_P7)) / 16.0F;

	return (uint32_t)(p_f * 1000.0F);
#endif // BMP280_CALC_TYPE
}

float BMP280_get_altitude(int32_t press)
{
	return 44330.0 * (1.0 - (float)(pow((1.0 * press) / init_press, 0.1903)));
}