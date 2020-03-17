#include "MPU9250.h"
#include "math.h"

// MPU9250 read/write operations
HAL_StatusTypeDef MPU9250_read(uint8_t *data)
{
	return HAL_I2C_Master_Receive(mpu9250.hi2c, (uint16_t)MPU9250_ADDRESS, (uint8_t *)data, 1, 1000);
}


HAL_StatusTypeDef MPU9250_write(uint8_t *data, uint8_t size)
{
	return HAL_I2C_Master_Transmit(mpu9250.hi2c, (uint16_t)MPU9250_ADDRESS, (uint8_t *)data, size, 1000);
}

HAL_StatusTypeDef MPU9250_read_reg(uint8_t address, uint8_t *data)
{
	uint8_t addr[1] = { address };
	HAL_StatusTypeDef status = MPU9250_write(addr, 1);
	
	if (status != HAL_OK)
		return HAL_ERROR;
	
	return MPU9250_read(data);
}

HAL_StatusTypeDef MPU9250_write_reg(uint8_t address, uint8_t data)
{
	uint8_t addr[2] = { address, data };
	return MPU9250_write(addr, 2);
}


// AK8963 read/write operations
HAL_StatusTypeDef AK8963_read(uint8_t *data)
{
	return HAL_I2C_Master_Receive(mpu9250.hi2c, (uint16_t)AK8963_ADDRESS, (uint8_t *)data, 1, 100);
}

HAL_StatusTypeDef AK8963_write(uint8_t *data, uint8_t size)
{
	return HAL_I2C_Master_Transmit(mpu9250.hi2c, (uint16_t)AK8963_ADDRESS, (uint8_t *)data, size, 100);
}

HAL_StatusTypeDef AK8963_read_reg(uint8_t address, uint8_t *data)
{
	uint8_t addr[1] = { address };
	HAL_StatusTypeDef status = AK8963_write(addr, 1);
	
	if (status != HAL_OK)
		return HAL_ERROR;
	
	return AK8963_read(data);
}

HAL_StatusTypeDef AK8963_write_reg(uint8_t address, uint8_t data)
{
	uint8_t addr[2] = { address, data };
	return AK8963_write(addr, 2);
}



// Functions

HAL_StatusTypeDef MPU9250_set_gyroscope_frequency(GYRO_BANDWIDTH bandwidth, GYRO_FREQUENCY frequency)
{
	switch (frequency)
	{
	case GYRO_FREQ_32KHZ:
		switch (bandwidth)
		{
		case GYRO_BANDWIDTH_DEFAULT:
		case GYRO_BANDWIDTH_250HZ:
		case GYRO_BANDWIDTH_184HZ:
		case GYRO_BANDWIDTH_92HZ:
		case GYRO_BANDWIDTH_41HZ:
		case GYRO_BANDWIDTH_20HZ:
		case GYRO_BANDWIDTH_10HZ:
		case GYRO_BANDWIDTH_5HZ:
			return HAL_ERROR;
		case GYRO_BANDWIDTH_8800HZ:
			MPU9250_write_reg(CONFIG, MPU9250_DLPF_CFG_0);
			MPU9250_write_reg(GYRO_CONFIG, GYRO_FCHOISE_2b00);
			return HAL_OK;
		case GYRO_BANDWIDTH_3600HZ:
			MPU9250_write_reg(CONFIG, MPU9250_DLPF_CFG_0);
			MPU9250_write_reg(GYRO_CONFIG, GYRO_FCHOISE_2b01);
			return HAL_OK;
		}
	case GYRO_FREQ_8KHZ:
		switch (bandwidth)
		{
		case GYRO_BANDWIDTH_DEFAULT:
		case GYRO_BANDWIDTH_8800HZ:
		case GYRO_BANDWIDTH_184HZ:
		case GYRO_BANDWIDTH_92HZ:
		case GYRO_BANDWIDTH_41HZ:
		case GYRO_BANDWIDTH_20HZ:
		case GYRO_BANDWIDTH_10HZ:
		case GYRO_BANDWIDTH_5HZ:
			return HAL_ERROR;
		case GYRO_BANDWIDTH_3600HZ:
			MPU9250_write_reg(CONFIG, MPU9250_DLPF_CFG_7);
			MPU9250_write_reg(GYRO_CONFIG, GYRO_FCHOISE_2b11);
			return HAL_OK;
		case GYRO_BANDWIDTH_250HZ:
			MPU9250_write_reg(CONFIG, MPU9250_DLPF_CFG_0);
			MPU9250_write_reg(GYRO_CONFIG, GYRO_FCHOISE_2b11);
			return HAL_OK;
		}
	case GYRO_FREQ_DEFAULT:
	case GYRO_FREQ_1KHZ:
		switch (bandwidth)
		{
		case GYRO_BANDWIDTH_8800HZ:
		case GYRO_BANDWIDTH_3600HZ:
		case GYRO_BANDWIDTH_250HZ:
			return HAL_ERROR;
		case GYRO_BANDWIDTH_184HZ:
			MPU9250_write_reg(CONFIG, MPU9250_DLPF_CFG_1);
			MPU9250_write_reg(GYRO_CONFIG, GYRO_FCHOISE_2b11);
			return HAL_OK;
		case GYRO_BANDWIDTH_92HZ:
			MPU9250_write_reg(CONFIG, MPU9250_DLPF_CFG_2);
			MPU9250_write_reg(GYRO_CONFIG, GYRO_FCHOISE_2b11);
			return HAL_OK;
		case GYRO_BANDWIDTH_DEFAULT:
		case GYRO_BANDWIDTH_41HZ:
			MPU9250_write_reg(CONFIG, MPU9250_DLPF_CFG_3);
			MPU9250_write_reg(GYRO_CONFIG, GYRO_FCHOISE_2b11);
			return HAL_OK;
		case GYRO_BANDWIDTH_20HZ:
			MPU9250_write_reg(CONFIG, MPU9250_DLPF_CFG_4);
			MPU9250_write_reg(GYRO_CONFIG, GYRO_FCHOISE_2b11);
			return HAL_OK;
		case GYRO_BANDWIDTH_10HZ:
			MPU9250_write_reg(CONFIG, MPU9250_DLPF_CFG_5);
			MPU9250_write_reg(GYRO_CONFIG, GYRO_FCHOISE_2b11);
			return HAL_OK;
		case GYRO_BANDWIDTH_5HZ:
			MPU9250_write_reg(CONFIG, MPU9250_DLPF_CFG_6);
			MPU9250_write_reg(GYRO_CONFIG, GYRO_FCHOISE_2b11);
			return HAL_OK;
		}
	}
}

HAL_StatusTypeDef MPU9250_set_accelerometer_frequency(ACCEL_BANDWIDTH bandwidth, ACCEL_FREQUENCY frequency)
{
	switch (frequency)
	{
	case ACCEL_FREQ_4KHZ:
		switch (bandwidth)
		{
		case ACCEL_BANDWIDTH_DEFAULT:
		case ACCEL_BANDWIDTH_420HZ:
		case ACCEL_BANDWIDTH_218HZ:
		case ACCEL_BANDWIDTH_99HZ:
		case ACCEL_BANDWIDTH_44HZ:
		case ACCEL_BANDWIDTH_21HZ:
		case ACCEL_BANDWIDTH_10HZ:
		case ACCEL_BANDWIDTH_5HZ:
			return HAL_ERROR;
		case ACCEL_BANDWIDTH_1046HZ:
			MPU9250_write_reg(ACCEL_CONFIG2, ACCEL_FCHOISE_0 | ACCEL_DLPF_CFG_0);
			return HAL_OK;
		}
	case ACCEL_FREQ_DEFAULT:
	case ACCEL_FREQ_1KHZ:
		switch (bandwidth)
		{
		case ACCEL_BANDWIDTH_1046HZ:
			return HAL_ERROR;
		case ACCEL_BANDWIDTH_420HZ:
			MPU9250_write_reg(ACCEL_CONFIG2, ACCEL_FCHOISE_1 | ACCEL_DLPF_CFG_7);
			return HAL_OK;
		case ACCEL_BANDWIDTH_218HZ:
			MPU9250_write_reg(ACCEL_CONFIG2, ACCEL_FCHOISE_1 | ACCEL_DLPF_CFG_0);
			return HAL_OK;
		case ACCEL_BANDWIDTH_99HZ:
			MPU9250_write_reg(ACCEL_CONFIG2, ACCEL_FCHOISE_1 | ACCEL_DLPF_CFG_2);
			return HAL_OK;
		case ACCEL_BANDWIDTH_DEFAULT:
		case ACCEL_BANDWIDTH_44HZ:
			MPU9250_write_reg(ACCEL_CONFIG2, ACCEL_FCHOISE_1 | ACCEL_DLPF_CFG_3);
			return HAL_OK;
		case ACCEL_BANDWIDTH_21HZ:
			MPU9250_write_reg(ACCEL_CONFIG2, ACCEL_FCHOISE_1 | ACCEL_DLPF_CFG_4);
			return HAL_OK;
		case ACCEL_BANDWIDTH_10HZ:
			MPU9250_write_reg(ACCEL_CONFIG2, ACCEL_FCHOISE_1 | ACCEL_DLPF_CFG_5);
			return HAL_OK;
		case ACCEL_BANDWIDTH_5HZ:
			MPU9250_write_reg(ACCEL_CONFIG2, ACCEL_FCHOISE_1 | ACCEL_DLPF_CFG_6);
			return HAL_OK;
		}
	}
}

HAL_StatusTypeDef MPU9250_set_sample_rate_divider(uint8_t divider)
{
	return MPU9250_write_reg(SMPLRT_DIV, divider);
}

HAL_StatusTypeDef MPU9250_set_internal_clock_source()
{
	return MPU9250_write_reg(PWR_MGMT_1, MPU9250_CLK_INTRNL);
}

HAL_StatusTypeDef MPU9250_set_PLL_clock_source()
{
	return MPU9250_write_reg(PWR_MGMT_1, MPU9250_CLK_PLL);
}

HAL_StatusTypeDef MPU9250_stop_clock_source()
{
	return MPU9250_write_reg(PWR_MGMT_1, MPU9250_CLK_STOP);
}

HAL_StatusTypeDef MPU9250_set_accel_fsr(ACCEL_SCALE scale)
{
	return MPU9250_write_reg(ACCEL_CONFIG, scale);
}

HAL_StatusTypeDef MPU9250_set_gyro_fsr(GYRO_SCALE scale)
{
	return MPU9250_write_reg(GYRO_CONFIG, scale);
}

HAL_StatusTypeDef MPU9250_set_mag_fsr(MAG_SCALE scale, MAG_FREQUENCY freq)
{
	return AK8963_write_reg(AK8963_CNTL, scale | freq);
}

HAL_StatusTypeDef MPU9250_reset() { 
	MPU9250_write_reg(PWR_MGMT_1, 0x80); 
	HAL_Delay(100);
	return HAL_OK;
}

HAL_StatusTypeDef MPU9250_who_am_i()
{
	uint8_t responce;
	MPU9250_read_reg(WHO_AM_I_MPU9250, &responce);
	return (responce == MPU9250_WHO_AM_I_RSP) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef MPU9250_startup(MPU9250_init_t *mpu9250_init)
{
	// Clear sleep mode bit (6), enable all sensors 
	MPU9250_write_reg(PWR_MGMT_1, 0x00);
	HAL_Delay(100);
	
	MPU9250_set_PLL_clock_source();

	if (MPU9250_set_gyroscope_frequency(mpu9250_init->gyro_bandwidth, mpu9250_init->gyro_freq) == HAL_ERROR)
		return HAL_ERROR;
	
	if (MPU9250_set_sample_rate_divider(0x04) == HAL_ERROR)
		return HAL_ERROR;
	
	if (MPU9250_set_gyro_fsr(mpu9250_init->gyro_scale) == HAL_ERROR)
		return HAL_ERROR;
  
	if (MPU9250_set_accel_fsr(mpu9250_init->accel_scale) == HAL_ERROR)
		return HAL_ERROR;

	if (MPU9250_set_accelerometer_frequency(mpu9250_init->accel_bandwidth, mpu9250_init->accel_freq) == HAL_ERROR)
		return HAL_ERROR;

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, 
	// enable I2C_BYPASS_EN so additional chips 
	// can join the I2C bus and all can be controlled by the Arduino as master
	MPU9250_write_reg(INT_PIN_CFG, 0x22);
	MPU9250_write_reg(INT_ENABLE, 0x00);																	// Enable data ready (bit 0) interrupt
	
	
	//if (MPU9250_self_test() == HAL_ERROR)
	//	return HAL_ERROR;
	
	return HAL_OK;
}

HAL_StatusTypeDef AK8963_startup(MPU9250_init_t *mpu9250_init)
{
	//if (AK8963_self_test() != HAL_OK)
	//	return HAL_ERROR;
	
	AK8963_calibrate();
	
	if (MPU9250_set_mag_fsr(mpu9250_init->mag_scale, mpu9250_init->mag_freq) == HAL_ERROR)
		return HAL_ERROR;
	HAL_Delay(50);
	
	return HAL_OK;
}


HAL_StatusTypeDef MPU9250_init(MPU9250_init_t *mpu9250_init)
{
	mpu9250.hi2c = mpu9250_init->hi2c;
	
	switch (mpu9250_init->accel_scale)
	{
	case AFS_2G:
		mpu9250.accelerometer.resolution	= 2.0 / 32768.0;
		break;
	case AFS_4G:
		mpu9250.accelerometer.resolution	= 4.0 / 32768.0;
		break;
	case AFS_8G:
		mpu9250.accelerometer.resolution	= 8.0 / 32768.0;
		break;
	case AFS_16G:
		mpu9250.accelerometer.resolution	= 16.0 / 32768.0;
		break;
	}
	
	switch (mpu9250_init->gyro_scale)
	{
	case GFS_250DPS:
		mpu9250.gyroscope.resolution		= 250.0 / 32768.0;
		break;
	case GFS_500DPS:
		mpu9250.gyroscope.resolution		= 500.0 / 32768.0;
		break;
	case GFS_1000DPS:
		mpu9250.gyroscope.resolution		= 1000.0 / 32768.0;
		break;
	case GFS_2000DPS:
		mpu9250.gyroscope.resolution		= 2000.0 / 32768.0;
		break;
	}
	
	switch (mpu9250_init->mag_scale)
	{
	case MFS_14BITS:
		mpu9250.magnetometer.resolution		= 0.6;
		break;
	case MFS_16BITS:
		mpu9250.magnetometer.resolution		= 0.15;
		break;
	}
	
	if (MPU9250_startup(mpu9250_init) == HAL_ERROR)
		return HAL_ERROR;
	return AK8963_startup(mpu9250_init);
}


void MPU9250_get_temp()
{
	uint8_t rawData[2];																						// x/y/z gyro register data stored here
	MPU9250_read_reg(TEMP_OUT_H, &rawData[1]);
	MPU9250_read_reg(TEMP_OUT_L, &rawData[0]);
	mpu9250.temperature = 21.0 + (((int16_t)(((int16_t)rawData[1]) << 8 | rawData[0])) - 4096) / 333.87;	// Turn the MSB and LSB into a 16-bit value
}

void MPU9250_get_accel()
{
	uint32_t t1 = HAL_GetTick();
	uint8_t rawData[6];
	MPU9250_read_reg(ACCEL_ZOUT_L, &rawData[5]);
	MPU9250_read_reg(ACCEL_ZOUT_H, &rawData[4]);
	MPU9250_read_reg(ACCEL_YOUT_L, &rawData[3]);
	MPU9250_read_reg(ACCEL_YOUT_H, &rawData[2]);
	MPU9250_read_reg(ACCEL_XOUT_L, &rawData[1]);
	MPU9250_read_reg(ACCEL_XOUT_H, &rawData[0]);
	uint32_t t2 = HAL_GetTick();
	uint8_t rawData1[6];
	
	mpu9250.accelerometer.data.x = (((int16_t)(((int16_t)rawData[0] << 8) | rawData[1])) * mpu9250.accelerometer.resolution) * 10 - mpu9250.accelerometer.offset.x;
	mpu9250.accelerometer.data.y = (((int16_t)(((int16_t)rawData[2] << 8) | rawData[3])) * mpu9250.accelerometer.resolution) * 10 - mpu9250.accelerometer.offset.y; 
	mpu9250.accelerometer.data.z = (((int16_t)(((int16_t)rawData[4] << 8) | rawData[5])) * mpu9250.accelerometer.resolution) * 10 - mpu9250.accelerometer.offset.z;
	
	uint32_t t4 = HAL_GetTick();
}

void MPU9250_get_gyro()
{
	uint8_t rawData[6];
	MPU9250_read_reg(GYRO_ZOUT_L, &rawData[5]);
	MPU9250_read_reg(GYRO_ZOUT_H, &rawData[4]);
	MPU9250_read_reg(GYRO_YOUT_L, &rawData[3]);
	MPU9250_read_reg(GYRO_YOUT_H, &rawData[2]);
	MPU9250_read_reg(GYRO_ZOUT_L, &rawData[1]);
	MPU9250_read_reg(GYRO_ZOUT_H, &rawData[0]);
	
	mpu9250.gyroscope.data.x = ((int16_t)(((int16_t)rawData[0] << 8) | rawData[1])) * mpu9250.gyroscope.resolution * M_PI / 180;
	mpu9250.gyroscope.data.y = ((int16_t)(((int16_t)rawData[2] << 8) | rawData[3])) * mpu9250.gyroscope.resolution * M_PI / 180;
	mpu9250.gyroscope.data.z = ((int16_t)(((int16_t)rawData[4] << 8) | rawData[5])) * mpu9250.gyroscope.resolution * M_PI / 180;
}

void MPU9250_get_mag()
{
	uint8_t rawData[7] = { 0 };																		// x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	uint8_t ready = 0;
	AK8963_read_reg(AK8963_ST1, &ready);
	if (ready & AK8963_DRDY) 
	{
		//ready = (uint8_t)0;
		// Read the six raw data and ST2 registers sequentially into data array
		AK8963_read_reg(AK8963_ZOUT_L, &rawData[5]);
		AK8963_read_reg(AK8963_ZOUT_H, &rawData[4]);
		AK8963_read_reg(AK8963_YOUT_L, &rawData[3]);
		AK8963_read_reg(AK8963_YOUT_H, &rawData[2]);
		AK8963_read_reg(AK8963_XOUT_L, &rawData[1]);
		AK8963_read_reg(AK8963_XOUT_H, &rawData[0]);
		AK8963_read_reg(AK8963_ST2, &rawData[6]);

		if (!(rawData[6] & AK8963_HOFL)) 
		{
			// Check if magnetic sensor overflow set, if not then report data
			// Without calibration matrix
			//mpu9250.magnetometer.data.x = ((int16_t)(((int16_t)rawData[0] << 8) | rawData[1])) * mpu9250.magnetometer.resolution * mpu9250.magnetometer.calibration.x - mpu9250.magnetometer.offset.x;
			//mpu9250.magnetometer.data.y = ((int16_t)(((int16_t)rawData[2] << 8) | rawData[3])) * mpu9250.magnetometer.resolution * mpu9250.magnetometer.calibration.y - mpu9250.magnetometer.offset.y;
			//mpu9250.magnetometer.data.z = ((int16_t)(((int16_t)rawData[4] << 8) | rawData[5])) * mpu9250.magnetometer.resolution * mpu9250.magnetometer.calibration.z - mpu9250.magnetometer.offset.z;
			
			// With calibration matrix
			
			float calibration_matrix[9] = { 
				1.217766, -0.112446, -0.079798,
				-0.112446, 1.449998, -0.081741,
				-0.079798, -0.081741, 1.444874
			};
			
			float bias[3] = { 95.723293, 61.318848, -115.324883 };
			
			float uncalibrated_values[3] = { 0, 0, 0 };
			uncalibrated_values[0] = ((int16_t)(((int16_t)rawData[0] << 8) | rawData[1])) * mpu9250.magnetometer.resolution * mpu9250.magnetometer.calibration.x - mpu9250.magnetometer.offset.x;
			uncalibrated_values[1] = ((int16_t)(((int16_t)rawData[2] << 8) | rawData[3])) * mpu9250.magnetometer.resolution * mpu9250.magnetometer.calibration.y - mpu9250.magnetometer.offset.y;
			uncalibrated_values[2] = ((int16_t)(((int16_t)rawData[4] << 8) | rawData[5])) * mpu9250.magnetometer.resolution * mpu9250.magnetometer.calibration.z - mpu9250.magnetometer.offset.z;
			
			for (int i = 0; i < 3; ++i) 
				uncalibrated_values[i] = uncalibrated_values[i] - bias[i];
			
			float result[3] = { 0, 0, 0 };
			
			for (int i = 0; i < 3; ++i)
				for (int j = 0; j < 3; ++j)
					result[i] += calibration_matrix[i * 3 + j] * uncalibrated_values[j];
			
			mpu9250.magnetometer.data.x = result[0];
			mpu9250.magnetometer.data.y = result[1];
			mpu9250.magnetometer.data.z = result[2];
		}
	}
}

void MPU9250_get_mag_raw(int16_t* raw)
{
	uint8_t rawData[7] = { 0 }; 																		// x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	uint8_t ready = 0;
	AK8963_read_reg(AK8963_ST1, &ready);
	if (ready & AK8963_DRDY) 
	{
		//ready = (uint8_t)0;
		// Read the six raw data and ST2 registers sequentially into data array
		AK8963_read_reg(AK8963_ZOUT_L, &rawData[5]);
		AK8963_read_reg(AK8963_ZOUT_H, &rawData[4]);
		AK8963_read_reg(AK8963_YOUT_L, &rawData[3]);
		AK8963_read_reg(AK8963_YOUT_H, &rawData[2]);
		AK8963_read_reg(AK8963_XOUT_L, &rawData[1]);
		AK8963_read_reg(AK8963_XOUT_H, &rawData[0]);
		AK8963_read_reg(AK8963_ST2, &rawData[6]);

		if (!(rawData[6] & AK8963_HOFL)) 
		{
			// Check if magnetic sensor overflow set, if not then report data
			raw[0] = ((int16_t)(((int16_t)rawData[0] << 8) | rawData[1]));
			raw[1] = ((int16_t)(((int16_t)rawData[2] << 8) | rawData[3]));
			raw[2] = ((int16_t)(((int16_t)rawData[4] << 8) | rawData[5]));
		}
	}
}

void MPU9250_calibrate()
{  
	uint8_t data[12];  // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };
  
	// reset device, reset all registers, clear gyro and accelerometer bias registers
	MPU9250_reset();
	HAL_Delay(100);  
   
	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	MPU9250_set_PLL_clock_source();
	MPU9250_write_reg(PWR_MGMT_2, 0x00); 
	HAL_Delay(200);  
  
	// Configure device for bias calculation
	MPU9250_write_reg(INT_ENABLE, 0x00);     // Disable all interrupts
	MPU9250_write_reg(FIFO_EN, 0x00);        // Disable FIFO
	MPU9250_write_reg(PWR_MGMT_1, 0x00);     // Turn on internal clock source
	MPU9250_write_reg(I2C_MST_CTRL, 0x00);   // Disable I2C master
	MPU9250_write_reg(USER_CTRL, 0x00);      // Disable FIFO and I2C master modes
	MPU9250_write_reg(USER_CTRL, 0x0C);      // Reset FIFO and DMP
	HAL_Delay(15);
  
	// Configure MPU9250 gyro and accelerometer for bias calculation
	MPU9250_write_reg(CONFIG, 0x01);         // Set low-pass filter to 188 Hz
	MPU9250_write_reg(SMPLRT_DIV, 0x00);    // Set sample rate to 1 kHz
	MPU9250_write_reg(GYRO_CONFIG, 0x00);    // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	MPU9250_write_reg(ACCEL_CONFIG, 0x00);   // Set accelerometer full-scale to 2 g, maximum sensitivity
 
	uint16_t  gyrosensitivity  = 131;    // = 131 LSB/degrees/sec
	uint16_t  accelsensitivity = 16384;   // = 16384 LSB/g

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	MPU9250_write_reg(USER_CTRL, 0x40);     // Enable FIFO  
	MPU9250_write_reg(FIFO_EN, 0x78);       // Enable gyro and accelerometer sensors for FIFO (max size 512 bytes in MPU-9250)
	HAL_Delay(40);  // accumulate 40 samples in 80 milliseconds = 480 bytes

	// At end of sample accumulation, turn off FIFO sensor read
	MPU9250_write_reg(FIFO_EN, 0x00);         // Disable gyro and accelerometer sensors for FIFO
	MPU9250_read_reg(FIFO_COUNTH, &data[0]);  
	MPU9250_read_reg(FIFO_COUNTL, &data[1]);  // read FIFO sample count
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count / 12; // How many sets of full gyro and accelerometer data for averaging

	for (ii = 0; ii < packet_count; ii++) {
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };
		MPU9250_read_reg(XA_OFFSET_H, &data[0]);					// read data for averaging
		MPU9250_read_reg(XA_OFFSET_L, &data[1]);
		MPU9250_read_reg(YA_OFFSET_H, &data[2]);
		MPU9250_read_reg(YA_OFFSET_L, &data[3]);
		MPU9250_read_reg(ZA_OFFSET_H, &data[4]);
		MPU9250_read_reg(ZA_OFFSET_L, &data[5]);
		MPU9250_read_reg(XG_OFFSET_H, &data[6]);
		MPU9250_read_reg(XG_OFFSET_L, &data[7]);
		MPU9250_read_reg(YG_OFFSET_H, &data[8]);
		MPU9250_read_reg(YG_OFFSET_L, &data[9]);
		MPU9250_read_reg(ZG_OFFSET_H, &data[10]);
		MPU9250_read_reg(ZG_OFFSET_L, &data[11]);
		accel_temp[0] = (int16_t)(((int16_t)data[0] << 8) | data[1]);   // Form signed 16-bit integer for each sample in FIFO
		accel_temp[1] = (int16_t)(((int16_t)data[2] << 8) | data[3]);
		accel_temp[2] = (int16_t)(((int16_t)data[4] << 8) | data[5]);    
		gyro_temp[0]  = (int16_t)(((int16_t)data[6] << 8) | data[7]);
		gyro_temp[1]  = (int16_t)(((int16_t)data[8] << 8) | data[9]);
		gyro_temp[2]  = (int16_t)(((int16_t)data[10] << 8) | data[11]);
    
		accel_bias[0] += (int32_t) accel_temp[0];  // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		accel_bias[1] += (int32_t) accel_temp[1];
		accel_bias[2] += (int32_t) accel_temp[2];
		gyro_bias[0]  += (int32_t) gyro_temp[0];
		gyro_bias[1]  += (int32_t) gyro_temp[1];
		gyro_bias[2]  += (int32_t) gyro_temp[2];
            
	}
	accel_bias[0] /= (int32_t) packet_count;  // Normalize sums to get average count biases
	accel_bias[1] /= (int32_t) packet_count;
	accel_bias[2] /= (int32_t) packet_count;
	gyro_bias[0]  /= (int32_t) packet_count;
	gyro_bias[1]  /= (int32_t) packet_count;
	gyro_bias[2]  /= (int32_t) packet_count;
    
	if (accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity; }  // Remove gravity from the z-axis accelerometer bias calculation
	else {accel_bias[2] += (int32_t) accelsensitivity; }
 
	// Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
	data[0] = (-gyro_bias[0] / 4  >> 8) & 0xFF;  // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data[1] = (-gyro_bias[0] / 4)       & 0xFF;  // Biases are additive, so change sign on calculated average gyro biases
	data[2] = (-gyro_bias[1] / 4  >> 8) & 0xFF;
	data[3] = (-gyro_bias[1] / 4)       & 0xFF;
	data[4] = (-gyro_bias[2] / 4  >> 8) & 0xFF;
	data[5] = (-gyro_bias[2] / 4)       & 0xFF;

	/// Push gyro biases to hardware registers
	/*  writeByte(MPU9250_ADDRESS, XG_OFFSET_H, data[0]);
	  writeByte(MPU9250_ADDRESS, XG_OFFSET_L, data[1]);
	  writeByte(MPU9250_ADDRESS, YG_OFFSET_H, data[2]);
	  writeByte(MPU9250_ADDRESS, YG_OFFSET_L, data[3]);
	  writeByte(MPU9250_ADDRESS, ZG_OFFSET_H, data[4]);
	  writeByte(MPU9250_ADDRESS, ZG_OFFSET_L, data[5]);
	  */
	mpu9250.gyroscope.offset.x = (float) gyro_bias[0] / (float) gyrosensitivity;  // construct gyro bias in deg/s for later manual subtraction
	mpu9250.gyroscope.offset.y = (float) gyro_bias[1] / (float) gyrosensitivity;
	mpu9250.gyroscope.offset.z = (float) gyro_bias[2] / (float) gyrosensitivity;

	// Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
	// factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
	// non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
	// compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
	// the accelerometer biases calculated above must be divided by 8.

	int32_t accel_bias_reg[3] = { 0, 0, 0 };  // A place to hold the factory accelerometer trim biases
	MPU9250_read_reg(XA_OFFSET_H, &data[0]);   // Read factory accelerometer trim values
	MPU9250_read_reg(XA_OFFSET_L, &data[1]); 
	accel_bias_reg[0] = (int16_t)((int16_t)data[0] << 8) | data[1];
	MPU9250_read_reg(YA_OFFSET_H, &data[0]);
	MPU9250_read_reg(YA_OFFSET_L, &data[1]);
	accel_bias_reg[1] = (int16_t)((int16_t)data[0] << 8) | data[1];
	MPU9250_read_reg(ZA_OFFSET_H, &data[0]);
	MPU9250_read_reg(ZA_OFFSET_L, &data[1]);
	accel_bias_reg[2] = (int16_t)((int16_t)data[0] << 8) | data[1];
  
	uint32_t mask = 1uL;  // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
	uint8_t mask_bit[3] = { 0, 0, 0 };  // Define array to hold mask bit for each accelerometer bias axis
  
	for(ii = 0 ; ii < 3 ; ii++) {
		if (accel_bias_reg[ii] & mask) mask_bit[ii] = 0x01;  // If temperature compensation bit is set, record that fact in mask_bit
	}

	// Construct total accelerometer bias, including calculated average accelerometer bias from above
	accel_bias_reg[0] -= (accel_bias[0] / 8);  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
	accel_bias_reg[1] -= (accel_bias[1] / 8);
	accel_bias_reg[2] -= (accel_bias[2] / 8);
 
	data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
	data[1] = (accel_bias_reg[0])      & 0xFF;
	data[1] = data[1] | mask_bit[0];  // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
	data[3] = (accel_bias_reg[1])      & 0xFF;
	data[3] = data[3] | mask_bit[1];  // preserve temperature compensation bit when writing back to accelerometer bias registers
	data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
	data[5] = (accel_bias_reg[2])      & 0xFF;
	data[5] = data[5] | mask_bit[2];  // preserve temperature compensation bit when writing back to accelerometer bias registers

	// Apparently this is not working for the acceleration biases in the MPU-9250
	// Are we handling the temperature correction bit properly?
	// Push accelerometer biases to hardware registers
	/*  writeByte(MPU9250_ADDRESS, XA_OFFSET_H, data[0]);
	  writeByte(MPU9250_ADDRESS, XA_OFFSET_L, data[1]);
	  writeByte(MPU9250_ADDRESS, YA_OFFSET_H, data[2]);
	  writeByte(MPU9250_ADDRESS, YA_OFFSET_L, data[3]);
	  writeByte(MPU9250_ADDRESS, ZA_OFFSET_H, data[4]);
	  writeByte(MPU9250_ADDRESS, ZA_OFFSET_L, data[5]);
	  */
	  // Output scaled accelerometer biases for manual subtraction in the main program
	mpu9250.accelerometer.offset.x = (float)accel_bias[0] / (float)accelsensitivity; 
	mpu9250.accelerometer.offset.y = (float)accel_bias[1] / (float)accelsensitivity;
	mpu9250.accelerometer.offset.z = (float)accel_bias[2] / (float)accelsensitivity;
}

void AK8963_calibrate()
{
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3]; 																					// x/y/z gyro calibration data stored here
	AK8963_write_reg(AK8963_CNTL, AK8963_POWER_DOWN); 														// Power down magnetometer  
	HAL_Delay(50);
	
	AK8963_write_reg(AK8963_CNTL, AK8963_FUSE_ROM); 														// Enter Fuse ROM access mode
	HAL_Delay(50);
	
	AK8963_read_reg(AK8963_ASAX, &rawData[0]); 															    // Read the x-, y-, and z-axis calibration values
	AK8963_read_reg(AK8963_ASAY, &rawData[1]);
	AK8963_read_reg(AK8963_ASAZ, &rawData[2]);
	
	mpu9250.magnetometer.calibration.x = (float)(rawData[0] - 128) / 256.0f + 1.0f;							// Return x-axis sensitivity adjustment values, etc.
	mpu9250.magnetometer.calibration.y = (float)(rawData[1] - 128) / 256.0f + 1.0f;  
	mpu9250.magnetometer.calibration.z = (float)(rawData[2] - 128) / 256.0f + 1.0f; 
	AK8963_write_reg(AK8963_CNTL, AK8963_POWER_DOWN);  														// Power down magnetometer  
	HAL_Delay(50);
}


HAL_StatusTypeDef MPU9250_self_test()
{
	float acc_delta[3] = { 0 };
	float gyr_delta[3] = { 0 };
	float acc_threshold[3] = {2.4, 2.9, 2.3};
	float gyr_threshold[3] = {2.6, 3.6, 1.7};
	
	
	uint8_t rawData[6] = { 0, 0, 0, 0, 0, 0 };
	uint8_t selfTest[6];
	int32_t gAvg[3] = { 0 }, aAvg[3] = { 0 }, aSTAvg[3] = { 0 }, gSTAvg[3] = { 0 };
	float factoryTrim[6];
	uint8_t FS = 0;
   
	MPU9250_write_reg(SMPLRT_DIV, 0x00);   // Set gyro sample rate to 1 kHz
	MPU9250_write_reg(CONFIG, 0x02);   // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
	MPU9250_write_reg(GYRO_CONFIG, FS << 3);   // Set full scale range for the gyro to 250 dps
	MPU9250_write_reg(ACCEL_CONFIG2, 0x02);   // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	MPU9250_write_reg(ACCEL_CONFIG, FS << 3);   // Set full scale range for the accelerometer to 2 g

	for(int ii = 0 ; ii < 200 ; ii++) {
		 // get average current values of gyro and acclerometer
  
		MPU9250_read_reg(ACCEL_XOUT_H, &rawData[0]);   // Read the six raw data registers into data array
		MPU9250_read_reg(ACCEL_XOUT_L, &rawData[1]); 
		MPU9250_read_reg(ACCEL_YOUT_H, &rawData[2]);
		MPU9250_read_reg(ACCEL_YOUT_L, &rawData[3]);
		MPU9250_read_reg(ACCEL_ZOUT_H, &rawData[4]);
		MPU9250_read_reg(ACCEL_ZOUT_L, &rawData[5]);
		
		aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
		aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
  
		MPU9250_read_reg(GYRO_XOUT_H, &rawData[0]);   // Read the six raw data registers sequentially into data array
		MPU9250_read_reg(GYRO_XOUT_L, &rawData[1]); 
		MPU9250_read_reg(GYRO_YOUT_H, &rawData[2]);
		MPU9250_read_reg(GYRO_YOUT_L, &rawData[3]);
		MPU9250_read_reg(GYRO_ZOUT_H, &rawData[4]);
		MPU9250_read_reg(GYRO_ZOUT_L, &rawData[5]);
		
		gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
		gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
	}
  
	for (int ii = 0; ii < 3; ii++) {
		 // Get average of 200 values and store as average current readings
		aAvg[ii] /= 200;
		gAvg[ii] /= 200;
	}
  
	// Configure the accelerometer for self-test
	MPU9250_write_reg(ACCEL_CONFIG, 0xE0);  // Enable self test on all three axes and set accelerometer range to +/- 2 g
	MPU9250_write_reg(GYRO_CONFIG, 0xE0);   // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
	HAL_Delay(25);  // Delay a while to let the device stabilize

	 for(int ii = 0 ; ii < 200 ; ii++) {
		 // get average self-test values of gyro and acclerometer
  
	 	MPU9250_read_reg(ACCEL_XOUT_H, &rawData[0]);    // Read the six raw data registers into data array
		MPU9250_read_reg(ACCEL_XOUT_L, &rawData[1]); 
		MPU9250_read_reg(ACCEL_YOUT_H, &rawData[2]);
		MPU9250_read_reg(ACCEL_YOUT_L, &rawData[3]);
		MPU9250_read_reg(ACCEL_ZOUT_H, &rawData[4]);
		MPU9250_read_reg(ACCEL_ZOUT_L, &rawData[5]);
		aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
		aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
  
		MPU9250_read_reg(GYRO_XOUT_H, &rawData[0]);    // Read the six raw data registers sequentially into data array
		MPU9250_read_reg(GYRO_XOUT_L, &rawData[1]); 
		MPU9250_read_reg(GYRO_YOUT_H, &rawData[2]);
		MPU9250_read_reg(GYRO_YOUT_L, &rawData[3]);
		MPU9250_read_reg(GYRO_ZOUT_H, &rawData[4]);
		MPU9250_read_reg(GYRO_ZOUT_L, &rawData[5]);
		gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);  // Turn the MSB and LSB into a signed 16-bit value
		gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
		gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
	}
  
	for (int ii = 0; ii < 3; ii++) {
		 // Get average of 200 values and store as average self-test readings
		aSTAvg[ii] /= 200;
		gSTAvg[ii] /= 200;
	}
  
	// Configure the gyro and accelerometer for normal operation
	MPU9250_write_reg(ACCEL_CONFIG, 0x00);
	MPU9250_write_reg(GYRO_CONFIG, 0x00);
	HAL_Delay(25);  // Delay a while to let the device stabilize
   
	// Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
	MPU9250_read_reg(SELF_TEST_X_ACCEL, &selfTest[0]);    // X-axis accel self-test results
	MPU9250_read_reg(SELF_TEST_Y_ACCEL, &selfTest[1]);    // Y-axis accel self-test results
	MPU9250_read_reg(SELF_TEST_Z_ACCEL, &selfTest[2]);    // Z-axis accel self-test results
	MPU9250_read_reg(SELF_TEST_X_GYRO, &selfTest[3]);    // X-axis gyro self-test results
	MPU9250_read_reg(SELF_TEST_Y_GYRO, &selfTest[4]);    // Y-axis gyro self-test results
	MPU9250_read_reg(SELF_TEST_Z_GYRO, &selfTest[5]);    // Z-axis gyro self-test results

   // Retrieve factory self-test value from self-test code reads
    factoryTrim[0] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[0] - 1.0)));  // FT[Xa] factory trim calculation
    factoryTrim[1] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[1] - 1.0)));  // FT[Ya] factory trim calculation
    factoryTrim[2] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[2] - 1.0)));  // FT[Za] factory trim calculation
    factoryTrim[3] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[3] - 1.0)));  // FT[Xg] factory trim calculation
    factoryTrim[4] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[4] - 1.0)));  // FT[Yg] factory trim calculation
    factoryTrim[5] = (float)(2620 / 1 << FS)*(pow(1.01, ((float)selfTest[5] - 1.0)));  // FT[Zg] factory trim calculation
 
   // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
   // To get percent, must multiply by 100
     for(int i = 0 ; i < 3 ; i++) {
		acc_delta[i] = 100.0*((float)(aSTAvg[i] - aAvg[i])) / factoryTrim[i] - 100.;		// Report percent differences
		gyr_delta[i] = 100.0*((float)(gSTAvg[i] - gAvg[i])) / factoryTrim[i + 3] - 100.;	// Report percent differences
	}
	
	for (int i = 0; i < 3; i++)
	{	
		if ((acc_threshold[i] < acc_delta[i]) || (acc_threshold[i] < -acc_delta[i]) || (gyr_threshold[i] < gyr_delta[i]) || (gyr_threshold[i] < -gyr_delta[i])) 
			return HAL_ERROR;
	}
	return HAL_OK;
}

HAL_StatusTypeDef AK8963_self_test()
{
	AK8963_write_reg(AK8963_CNTL, AK8963_POWER_DOWN);
	HAL_Delay(50);
	
	AK8963_write_reg(AK8963_ASTC, AK8963_ASTC_SELF);
	
	AK8963_write_reg(AK8963_CNTL, AK8963_SELF_TEST);
	
	uint8_t drdy;
	int16_t sensor_data[3] = { 0 };
	
	for (uint8_t i = 0; i < 100; i++)
	{
		AK8963_read_reg(AK8963_ST1, &drdy);
		if (drdy & AK8963_DRDY)
		{
			uint8_t rawData[7];
			AK8963_read_reg(AK8963_ZOUT_L, &rawData[5]);
			AK8963_read_reg(AK8963_ZOUT_H, &rawData[4]);
			AK8963_read_reg(AK8963_YOUT_L, &rawData[3]);
			AK8963_read_reg(AK8963_YOUT_H, &rawData[2]);
			AK8963_read_reg(AK8963_XOUT_L, &rawData[1]);
			AK8963_read_reg(AK8963_XOUT_H, &rawData[0]);
			AK8963_read_reg(AK8963_ST2, &rawData[6]);
			
			int16_t threshold_min[3] = { 0 };
			int16_t threshold_max[3] = { 0 };
			uint8_t bit;
			AK8963_read_reg(AK8963_CNTL, &bit);
			if (bit & 0x10)
			{
				threshold_min[0] = -200;
				threshold_min[1] = -200;
				threshold_min[2] = -3200;
					
				threshold_max[0] = 200;
				threshold_max[1] = 200;
				threshold_max[2] = -800;
			}
			else 
			{
				threshold_min[0] = -50;
				threshold_min[1] = -50;
				threshold_min[2] = -800;
					
				threshold_max[0] = 50;
				threshold_max[1] = 50;
				threshold_max[2] = -200;
			}
			
			sensor_data[0] = (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]);
			sensor_data[1] = (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]);
			sensor_data[2] = (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]);
			
			for (uint8_t k = 0; k < 3; k++)
			{
				if (sensor_data[k] < threshold_min[k] || sensor_data[k] > threshold_max[k])
				{
					AK8963_write_reg(AK8963_ASTC, 0x00);
					AK8963_write_reg(AK8963_CNTL, AK8963_POWER_DOWN);
					return HAL_ERROR;
				}
			}
			AK8963_write_reg(AK8963_ASTC, 0x00);
			AK8963_write_reg(AK8963_CNTL, AK8963_POWER_DOWN);
			HAL_Delay(100);
			return HAL_OK;
			
		}
	}
	
	AK8963_write_reg(AK8963_ASTC, 0x00);
	AK8963_write_reg(AK8963_CNTL, AK8963_POWER_DOWN);
	return HAL_TIMEOUT;
}
