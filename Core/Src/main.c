/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "ssd1306.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RAD_TO_DEG (180.0/M_PI)
//#define SCALE_Q(n) (1.0 / (1 << n))
#define SCALE_Q(n) (pow(0.5, n))
//#define BNO_I2C_ADDRESS (0x4A << 1) // Adafruit
#define BNO_I2C_ADDRESS (0x4B << 1) // SparkFun BNO086
#define BNO_I2C_HANDLE &hi2c1
#define BNO_MSG_LENGTH 21
#define BNO_READ_PERIOD 20 // ms
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t START_BNO_STABILIZED_ROTATION_VECTOR_100_HZ[/* BNO_MSG_LENGTH */] =
{ 0x15, 0x00, 0x02, 0x00, 0xFD, 0x28, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00,
		0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
/* https://www.ceva-ip.com/wp-content/uploads/2019/10/SH-2-Reference-Manual.pdf
 * 6.5.4 Set Feature Command (0xFD)
 * Hint: 0x00002710 = 10000 us (100 Hz)
 */

uint8_t FishedOutMessage[BNO_MSG_LENGTH];
uint8_t BnoRxBuff[BNO_MSG_LENGTH];
int16_t Data1;
int16_t Data2;
int16_t Data3;
int16_t Data4;

double Yaw, Pitch, Roll;
double Qi, Qj, Qk, Qr;

char lcd_line[64];
uint32_t BnoSoftTimer;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

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
  MX_LPUART1_UART_Init();
  MX_I2C1_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_SetCursor(20, 0);
	ssd1306_WriteString("ufnalski.edu.pl", Font_6x8, White);
	ssd1306_SetCursor(7, 11);
	ssd1306_WriteString("BNO085/86 9-DoF AHRS", Font_6x8, White);
	ssd1306_UpdateScreen();

	HAL_I2C_Master_Transmit(BNO_I2C_HANDLE, BNO_I2C_ADDRESS,
			START_BNO_STABILIZED_ROTATION_VECTOR_100_HZ,
			sizeof(START_BNO_STABILIZED_ROTATION_VECTOR_100_HZ), 10);
	HAL_Delay(100);

	BnoSoftTimer = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{

		if (HAL_GetTick() - BnoSoftTimer > BNO_READ_PERIOD)
		{
			BnoSoftTimer = HAL_GetTick();

			HAL_I2C_Master_Receive(BNO_I2C_HANDLE, BNO_I2C_ADDRESS, BnoRxBuff,
					BNO_MSG_LENGTH, 10);

			if ((BnoRxBuff[9] == 0x28))
			/* https://www.ceva-ip.com/wp-content/uploads/2019/10/SH-2-Reference-Manual.pdf
			 * 6.5.42 ARVR-Stabilized Rotation Vector (0x28)
			 */
			{
				memcpy(FishedOutMessage, BnoRxBuff, BNO_MSG_LENGTH);
				Data1 = (((uint16_t) FishedOutMessage[14]) << 8)
						| FishedOutMessage[13];
				Data2 = (((uint16_t) FishedOutMessage[16]) << 8)
						| FishedOutMessage[15];
				Data3 = (((uint16_t) FishedOutMessage[18]) << 8)
						| FishedOutMessage[17];
				Data4 = (((uint16_t) FishedOutMessage[20]) << 8)
						| FishedOutMessage[19];

				Qi = ((double) Data2) * SCALE_Q(14);
				Qj = ((double) Data3) * SCALE_Q(14);
				Qk = ((double) Data4) * SCALE_Q(14);
				Qr = ((double) Data1) * SCALE_Q(14);

				Yaw = atan2(2.0 * (Qi * Qj + Qk * Qr),
						(Qi * Qi - Qj * Qj - Qk * Qk + Qr * Qr)) * RAD_TO_DEG;
				Pitch =
						asin(
								-2.0 * (Qi * Qk - Qj * Qr)
										/ (Qi * Qi + Qj * Qj + Qk * Qk + Qr * Qr)) * RAD_TO_DEG;
				Roll = atan2(2.0 * (Qj * Qk + Qi * Qr),
						(-Qi * Qi - Qj * Qj + Qk * Qk + Qr * Qr)) * RAD_TO_DEG;

				ssd1306_SetCursor(5, 25);
				sprintf(lcd_line, "Heading: %7.2f deg  ", Roll);
				ssd1306_WriteString(lcd_line, Font_6x8, White);

				ssd1306_SetCursor(5, 37);
				sprintf(lcd_line, "Roll   : %7.2f deg  ", Yaw);
				ssd1306_WriteString(lcd_line, Font_6x8, White);

				ssd1306_SetCursor(5, 49);
				sprintf(lcd_line, "Pitch  : %7.2f deg  ", Pitch);
				ssd1306_WriteString(lcd_line, Font_6x8, White);
				ssd1306_UpdateScreen(); // this single line alone probably adds more than BNO_READ_PERIOD :)
			}
		}

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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
