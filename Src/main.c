
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2020 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */

#include "MPU9250.h"
#include "BMP280.h"
#include "nRF24L01.h"
#include "battery.h"
#include "radio_handler.h"
#include "motor.h"
#include "copter.h"
#include "math.h"
#include <stdlib.h>
#include "accel_calibration.h"
#include "mag_calibration.h"


/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


extern battery_t battery;

//extern bt_message_t message;

extern copter_t copter;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/



/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void i2c_scan(I2C_HandleTypeDef *hi2c, uint8_t* addr)
{
	uint8_t ptr = 0;
	for (uint8_t i = 1; i < 255; i++)
	{
		uint8_t data = 0;
		if (HAL_I2C_Master_Transmit(hi2c, (uint16_t)i, (uint8_t *)&data, 1, 1000) == HAL_OK)
		{
			addr[ptr++] = i;
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	MX_TIM2_Init();
	MX_I2C2_Init();
	MX_SPI1_Init();
	MX_TIM4_Init();
	/* USER CODE BEGIN 2 */

	// Sensors init
	//     MPU9250
	HAL_Delay(50);
	
	HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
	
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	htim4.Instance->CCR1 = 10;
	htim4.Instance->CCR2 = 10;
	htim4.Instance->CCR3 = 10;
	
	
	uint8_t addr[10] = { 0 };
	i2c_scan(&hi2c2, addr);
	
	MPU9250_init_t mpu9250_init_s = 
	{ 
		.hi2c = &hi2c2,
		.accel_scale = AFS_2G,
		.gyro_scale = GFS_250DPS,
		.mag_scale = MFS_16BITS,
		.mag_freq = MAG_FREQ_100HZ,
		.accel_bandwidth = ACCEL_BANDWIDTH_DEFAULT,
		.accel_freq = ACCEL_FREQ_DEFAULT,
		.gyro_bandwidth = GYRO_BANDWIDTH_DEFAULT,
		.gyro_freq = GYRO_FREQ_DEFAULT
	};
	HAL_StatusTypeDef ok = MPU9250_init(&mpu9250_init_s);
	//accelerometer_calibration();
	//mag_calibration();
	
	
	//save_calibration_to_flash();
	//load_calibration_from_flash();
	
	//     BMP280
	BMP280_init_t bmp280_init = 
	{ 
		.hi2c = &hi2c2,
		.mode = BMP280_NORMAL_MODE,
		.filter_coeff = BMP280_FILTER_COEF_16,
		.ost = BMP280_TEMP_LOW_POWER,
		.osp = BMP280_PRESS_ULTRA_HIGH_RES,
		.stby_time = BMP280_STANDBY_05_MS
	};
	
	ok = BMP280_init(&bmp280_init);
	
	// BT connectivity init

	
	
	// Motor Init
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	copter_init(&htim1, TIM_CHANNEL_1, TIM_CHANNEL_4, TIM_CHANNEL_3, TIM_CHANNEL_2);
	
	
	// Battery Level Measurement init
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*) &battery.buf, ADC_BUFFER_SIZE);
	
	
	// Sheduler timer init
	HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
	
	
	// nRF24 radio init & config
	nRF24_Init();
	uint8_t nrf_pres = nRF24_Check();
	uint8_t ADDR[] = { 'n', 'R', 'F', '2', '4' };
	
	nRF24_EnableAutoAck();
	nRF24_SetRFChannel(120);
	nRF24_SetDataRate(nRF24_DR_2Mbps);
	nRF24_SetCRCScheme(nRF24_CRC_off);
	nRF24_SetAddrWidth(5);
	
	nRF24_SetAddr(nRF24_PIPE1, ADDR);
	nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_ON, 32);
	
	nRF24_SetTXPower(nRF24_TXPWR_0dBm);
	nRF24_SetOperationalMode(nRF24_MODE_RX);
	nRF24_SetPowerMode(nRF24_PWR_UP);
	
	uint8_t ack_payload[32] = {0};
	nRF24_WriteAckPayload(nRF24_PIPE1, ack_payload, 32);
	nRF24_CE_H;  // start receiving
	
	
	htim4.Instance->CCR2 = 0;
	htim4.Instance->CCR3 = 0;
	htim4.Instance->CCR1 = 11; 
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {


  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV8;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV8;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
