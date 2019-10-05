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
	return HAL_I2C_Master_Receive(mpu9250.hi2c, (uint16_t)AK8963_ADDRESS, (uint8_t *)data, 1, 1000);
}

HAL_StatusTypeDef AK8963_write(uint8_t *data, uint8_t size)
{
	return HAL_I2C_Master_Transmit(mpu9250.hi2c, (uint16_t)AK8963_ADDRESS, (uint8_t *)data, size, 1000);
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



// Fuctions
HAL_StatusTypeDef MPU9250_who_am_i()
{
	uint8_t responce;
	MPU9250_read_reg(WHO_AM_I_MPU9250, &responce);
	return (responce == 0x71) ? HAL_OK : HAL_ERROR;
}

void MPU9250_startup()
{  																					
	MPU9250_write_reg(PWR_MGMT_1, 0x00);																	// Clear sleep mode bit (6), enable all sensors 
	HAL_Delay(100);																							// Delay 100 ms for PLL to get established on x-axis gyro; should check for PLL ready interrupt  
	
	MPU9250_write_reg(PWR_MGMT_1, 0x01);																	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001

	// Configure Gyro and Accelerometer
	// Disable FSYNC and set accelerometer and gyro bandwidth to 44 and 42 Hz, respectively; 
	// DLPF_CFG = bits 2:0 = 010; this sets the sample rate at 1 kHz for both
	// Maximum delay is 4.9 ms which is just over a 200 Hz maximum rate
	MPU9250_write_reg(CONFIG, 0x03);
	
	// Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
	MPU9250_write_reg(SMPLRT_DIV, 0x04);																	// Use a 200 Hz rate; the same rate set in CONFIG above
	
	// Set gyroscope full scale range
	// Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
	uint8_t gyro_config;
	MPU9250_read_reg(GYRO_CONFIG, &gyro_config);
	gyro_config &= 0b11100100;																				// Clear Fchoice bits [1:0] and AFS bits [4:3]
	gyro_config |= (mpu9250.gyroscope.scale << 3);															// Set full scale range for the gyro
	MPU9250_write_reg(GYRO_CONFIG, gyro_config);															// Write new GYRO_CONFIG value to register
  
	// Set accelerometer full-scale range configuration
	uint8_t acc_config;																						// Get current ACCEL_CONFIG register value
	MPU9250_read_reg(ACCEL_CONFIG, &acc_config);
	acc_config &= 0b11100111;																				// Clear AFS bits [4:3]
	acc_config |= (mpu9250.accelerometer.scale << 3); 														// Set full scale range for the accelerometer 
	MPU9250_write_reg(ACCEL_CONFIG, acc_config);															// Write new ACCEL_CONFIG register value

	// Set accelerometer sample rate configuration
	// It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
	// accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
	uint8_t acc_config2;
	MPU9250_read_reg(ACCEL_CONFIG2, &acc_config2);															// Get current ACCEL_CONFIG2 register value
	acc_config2 &= 0b11110000;																				// Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])  
	acc_config2 |= 0b00000011;																				// Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
	MPU9250_write_reg(ACCEL_CONFIG2, acc_config2);															// Write new ACCEL_CONFIG2 register value

	// The accelerometer, gyro, and thermometer are set to 1 kHz sample rates, 
	// but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

	// Configure Interrupts and Bypass Enable
	// Set interrupt pin active high, push-pull, and clear on read of INT_STATUS, 
	// enable I2C_BYPASS_EN so additional chips 
	// can join the I2C bus and all can be controlled by the Arduino as master
	MPU9250_write_reg(INT_PIN_CFG, 0x22);
	MPU9250_write_reg(INT_ENABLE, 0x00);																	// Enable data ready (bit 0) interrupt
}

void AK8963_startup()
{
	// First extract the factory calibration for each magnetometer axis
	uint8_t rawData[3];																						// x/y/z gyro calibration data stored here
	AK8963_write_reg(AK8963_CNTL, 0x00);																	// Power down magnetometer  
	HAL_Delay(50);
	
	AK8963_write_reg(AK8963_CNTL, 0x0F);																	// Enter Fuse ROM access mode
	HAL_Delay(50);
	
	AK8963_read_reg(AK8963_ASAX, &rawData[0]);															   // Read the x-, y-, and z-axis calibration values
	AK8963_read_reg(AK8963_ASAY, &rawData[1]);
	AK8963_read_reg(AK8963_ASAZ, &rawData[2]);
	
	//destination[0] =  (float)(rawData[0] - 128) / 256.0f + 1.0f;											// Return x-axis sensitivity adjustment values, etc.
	//destination[1] =  (float)(rawData[1] - 128) / 256.0f + 1.0f;  
	//destination[2] =  (float)(rawData[2] - 128) / 256.0f + 1.0f; 
	AK8963_write_reg(AK8963_CNTL, 0x00);																	// Power down magnetometer  
	HAL_Delay(50);
	// Configure the magnetometer for continuous read and highest resolution
	// set Mscale bit 4 to 1 (0) to enable 16 (14) bit resolution in CNTL register,
	// and enable continuous mode data acquisition Mmode (bits [3:0]), 0010 for 8 Hz and 0110 for 100 Hz sample rates
	AK8963_write_reg(AK8963_CNTL, (uint8_t)((mpu9250.magnetometer.scale << 4) | 0b00000110));				// Set magnetometer data resolution and sample ODR
	HAL_Delay(50);
}


void MPU9250_init(I2C_HandleTypeDef *hi2c)
{
	mpu9250.hi2c = hi2c;
	
	mpu9250.accelerometer.scale			= AFS_2G;
	mpu9250.gyroscope.scale				= GFS_250DPS;
	mpu9250.magnetometer.scale			= MFS_16BITS;
	
	mpu9250.accelerometer.resolution	= 2.0 / 32768.0;
	mpu9250.gyroscope.resolution		= 250.0 / 32768.0;
	mpu9250.magnetometer.resolution		= 0.15;
	
	MPU9250_startup();
	AK8963_startup();
}


void resetMPU9250() { 
	MPU9250_write_reg(PWR_MGMT_1, 0x80); 																	// Write a one to bit 7 reset bit; toggle reset device
	HAL_Delay(100);
}

void MPU9250_get_temp()
{
	uint8_t rawData[2];																						// x/y/z gyro register data stored here
	MPU9250_read_reg(TEMP_OUT_H, &rawData[1]);
	MPU9250_read_reg(TEMP_OUT_L, &rawData[0]);
	mpu9250.temperature = (int16_t)(((int16_t)rawData[1]) << 8 | rawData[0]);						// Turn the MSB and LSB into a 16-bit value
}

void MPU9250_get_accel()
{
	uint8_t rawData[6];
	MPU9250_read_reg(ACCEL_ZOUT_L, &rawData[5]);
	MPU9250_read_reg(ACCEL_ZOUT_H, &rawData[4]);
	MPU9250_read_reg(ACCEL_YOUT_L, &rawData[3]);
	MPU9250_read_reg(ACCEL_YOUT_H, &rawData[2]);
	MPU9250_read_reg(ACCEL_XOUT_L, &rawData[1]);
	MPU9250_read_reg(ACCEL_XOUT_H, &rawData[0]);
	
	mpu9250.accelerometer.data.x = (((int16_t)(((int16_t)rawData[0] << 8) | rawData[1])) * mpu9250.accelerometer.resolution) * 10 - mpu9250.accelerometer.offset.x;
	mpu9250.accelerometer.data.y = (((int16_t)(((int16_t)rawData[2] << 8) | rawData[3])) * mpu9250.accelerometer.resolution) * 10 - mpu9250.accelerometer.offset.y; 
	mpu9250.accelerometer.data.z = (((int16_t)(((int16_t)rawData[4] << 8) | rawData[5])) * mpu9250.accelerometer.resolution) * 10 - mpu9250.accelerometer.offset.z;
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
	
	mpu9250.gyroscope.data.x = ((int16_t)(((int16_t)rawData[0] << 8) | rawData[1])) * mpu9250.gyroscope.resolution;
	mpu9250.gyroscope.data.y = ((int16_t)(((int16_t)rawData[2] << 8) | rawData[3])) * mpu9250.gyroscope.resolution;
	mpu9250.gyroscope.data.z = ((int16_t)(((int16_t)rawData[4] << 8) | rawData[5])) * mpu9250.gyroscope.resolution;
}

void MPU9250_get_mag()
{
	uint8_t rawData[7];																		// x/y/z gyro register data, ST2 register stored here, must read ST2 at end of data acquisition
	uint8_t resp;
	AK8963_read_reg(AK8963_ST1, &resp);
	if(resp & 0x01) 
	{
		resp = (uint8_t)0;
		// Read the six raw data and ST2 registers sequentially into data array
		AK8963_read_reg(AK8963_ST2, &rawData[6]);
		AK8963_read_reg(AK8963_ZOUT_L, &rawData[5]);
		AK8963_read_reg(AK8963_ZOUT_H, &rawData[4]);
		AK8963_read_reg(AK8963_YOUT_L, &rawData[3]);
		AK8963_read_reg(AK8963_YOUT_H, &rawData[2]);
		AK8963_read_reg(AK8963_XOUT_L, &rawData[1]);
		AK8963_read_reg(AK8963_XOUT_H, &rawData[0]);

		if (!(rawData[6] & 0x08)) 
		{
			 // Check if magnetic sensor overflow set, if not then report data
			mpu9250.magnetometer.data.x = ((int16_t)(((int16_t)rawData[0] << 8) | rawData[1])) * mpu9250.magnetometer.resolution;
			mpu9250.magnetometer.data.y = ((int16_t)(((int16_t)rawData[2] << 8) | rawData[3])) * mpu9250.magnetometer.resolution;
			mpu9250.magnetometer.data.z = ((int16_t)(((int16_t)rawData[4] << 8) | rawData[5])) * mpu9250.magnetometer.resolution;
		}
	}
}

void MPU9250_calibrate()
{  
	uint8_t data[12];  // data array to hold accelerometer and gyro x, y, z, data
	uint16_t ii, packet_count, fifo_count;
	int32_t gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };
  
	// reset device, reset all registers, clear gyro and accelerometer bias registers
	MPU9250_write_reg(PWR_MGMT_1, 0x80);																	// Write a one to bit 7 reset bit; toggle reset device
	HAL_Delay(100);  
   
	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	MPU9250_write_reg(PWR_MGMT_1, 0x01);  
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


HAL_StatusTypeDef MPU9250_self_test()
{
	float acc_delta[3] = { 0 };
	float gyr_delta[3] = { 0 };
	float acc_threshold[3] = {2.4, 2.9, 2.3};
	float gyr_threshold[3] = {2.3, 3.6, 1.7};
	
	
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
