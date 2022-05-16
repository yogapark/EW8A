/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "sc8915.h"
#include "OLED_SSD1306.h"
#include <stdio.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint16_t ccr = 0;
volatile uint16_t adcval[8];
char RcvData[32];
char* currentline;
int nError = 0;
int gCount_SCREEN_IDLE_TIME = 0;
int gCount_FullMode_setting_Time = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//void SC8915_W_Command(uint8_t cmd , uint8_t data);
//void SC8915_R_DATA(uint8_t reg, uint8_t* data_buffer, uint16_t buffer_size);

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
  int count_acc = 0;
  int i;
//  uint8_t page,column;
  uint8_t read_buff[10]={0};
  int mV_VBUS = 0,mV_VBAT = 0,mA_IBUS = 0,mA_IBAT = 0;
  currentline = RcvData;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_TIM14_Init();
  MX_TIM16_Init();
  MX_ADC_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim14 , TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim16 , TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim14); // 10ms interrupt
  HAL_TIM_Base_Start_IT(&htim6);  // 1s interrupt


  IN_EN1_voltage_Off();
  IN_EN2_voltage_Off();
  OUT_EN_voltage_Off();
  SC8915_PSTOP_Off();

  OLED_Init();
  OLED_Screen_Clear();

  TIM16->CCR1 = 0;

//  SC8915_init();
  HAL_Delay(10);

//  OLED_image_test_ener();
//  OLED_image_test_iROAD();
  OLED_image_Loading();
//  HAL_Delay(1000);
  OLED_Screen_Clear();
  OLED_SetScreenOn(); // 2021.04.05
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

#if 1
	if(isSwitchFullMode())
	{
		OLED_SCREEN_ON();
		if( is_IN_EN_ON() )
		{
			if (gCount_FullMode_setting_Time >= FULLMODE_ACTION_TEME)
				SC8915_FULL_mode();
			else
				SC8915_HALF_mode();

			HAL_Delay(10);
			SC8915_R_DATA(0x0D, &read_buff[0],10);
			HAL_Delay(10);

//			if(isSwitchFullMode())
				gCount_SCREEN_IDLE_TIME = 0;

		}
		else if(isBATon())
		{
			SC8915_HALF_mode();
			HAL_Delay(10);
			SC8915_R_DATA(0x0D, &read_buff[0],10);
			HAL_Delay(10);

			if(isSwitchFullMode())
				gCount_SCREEN_IDLE_TIME = 0;
		}
		else
		{
			for(i=0;i<10;i++)
				read_buff[i]=0;
		}
		Set_OUT_EN_ON();
//		OLED_display_string(0, 54, "FULL");
//		OLED_Screen_Clean(2, 54, 24);
	}
	else //if(isSwitchHalfMode())
	{
		OLED_SCREEN_ON();
		if( is_IN_EN_ON() )
		{
			SC8915_HALF_mode();
			HAL_Delay(10);
			SC8915_R_DATA(0x0D, &read_buff[0],10);
			HAL_Delay(10);
			gCount_SCREEN_IDLE_TIME = 0;
		}
		else if(isBATon())
		{
			SC8915_HALF_mode();
			HAL_Delay(10);
			SC8915_R_DATA(0x0D, &read_buff[0],10);
			HAL_Delay(10);
		}
		else
		{
			for(i=0;i<10;i++)
				read_buff[i]=0;
		}
		Set_OUT_EN_ON();
//		OLED_display_string(2, 54, "HALF");
//		OLED_Screen_Clean(0, 54, 24);
	}
//	else
//	{
//		OLED_SCREEN_OFF();
//		SC8915_PSTOP_Off();
//		count_acc  = 0;
//
//		Set_OUT_EN_OFF();
//		HAL_Delay(1000);
//		continue;
//	}
//
#endif



//	SC8915_R_DATA(0x00, &read_buff[0],1);
//	SC8915_R_DATA(0x0D, &read_buff[0],10);

//	HAL_Delay(100);

//	SC8915_R_DATA(0x00, &read_buff[0],1);
//
	// Display VBUS
//	OLED_display_string(1, 49, "I");
	mV_VBUS = VBUS_mV_converter(read_buff[0],read_buff[1] );
//	OLED_display_V_Num11x16(1, 56, mV_VBUS);

	// Display IBUS
//	mA_IBUS = IBUS_mA_converter(read_buff[4],read_buff[5]);
//	OLED_display_A(2, 102, mA_IBUS);

	// display VBAT
//	OLED_display_string(3, 49, "O");
	mV_VBAT = VBAT_mV_converter(read_buff[2],read_buff[3]);
	OLED_display_V_Num11x16(2, 86, mV_VBAT);

	// Display IBAT
	mA_IBAT = IBAT_mA_converter(read_buff[6],read_buff[7]);
	OLED_display_A(5, 102, mA_IBAT);


	// display MCU ADC data.
//	if(isSwitchFullMode())
	if(1)
	{
		OLED_display_string(2, 0, "ACC");
		sprintf(currentline,"%03d",mcuADCto_mV(adcval[0]));
		OLED_display_string_100m(2,20,currentline);

		OLED_display_string(0, 0, "VCC");
		sprintf(currentline,"%03d",mcuADCto_mV(adcval[1]));
		OLED_display_string_100m(0,20,currentline);
	}
	else// if(isSwitchHalfMode())
	{
		OLED_display_string(0, 0, "Acc2");
		sprintf(currentline,"%03d",mcuADCto_mV(adcval[5]));
		OLED_display_string_100m(0,20,currentline);

		OLED_display_string(2, 0, "Vin2");
		sprintf(currentline,"%03d",mcuADCto_mV(adcval[2]));
		OLED_display_string_100m(2,26,currentline);
	}

//	OLED_display_string(4, 0, "Vout");
//	sprintf(currentline,"%03d",mcuADCto_mV(adcval[3]));
//	OLED_display_string_100m(4,26,currentline);

	OLED_display_string(0, 108, "BAT");

//	OLED_display_string(7, 49, "v");
//	sprintf(currentline,"%03d",mcuADCto_mV(adcval[2]));
//	OLED_display_string_100m(7,55,currentline);

	// temperature

#if 1
	OLED_display_string(4, 0, "TEM");
//	sprintf(currentline,"%03d",adcval[4]/10);
	sprintf(currentline,"%+03d",/*-2 );*/mcuTemperature(adcval[6]));
//	OLED_display_string_Tem(0,109,currentline);
	OLED_display_string_Tem(4,20,currentline);
#else

	//	sprintf(currentline,"%03d",adcval[4]/10);
		sprintf(currentline,"%+03d",/*-2 );*/mcuTemperature2(adcval[6]));
	//	OLED_display_string_Tem(0,109,currentline);
		OLED_display_string_Tem(3,50,currentline);

//		sprintf(currentline,"%04d",adcval[6]);
	//	sprintf(currentline,"%+03d",/*-2 );*/mcuTemperature(adcval[6]));
//		OLED_display_string(7,50,currentline);
	//	OLED_display_string_Tem(7,50,currentline);

		sprintf(currentline,"%04d",*((uint16_t*) ((uint32_t)0x1FFFF7B8)));
		OLED_display_string(5,50,currentline);

#endif



	OLED_display_Battery_Icon_wide(6,0,mV_VBAT);

	OLED_display_Version(8,90,nVERSION);

	OLED_display_live_action(8,83);

	if(mV_VBUS > 11400)  // 2021.12.08 v2.03 (11400 -> 11200) v2.04 return.
//		if(I_VALUE > 9000)
	{
		count_acc++;
		if(count_acc > 5)
		{
			count_acc=6;
			SC8915_PSTOP_On();
		}
	}
	else
	{
		count_acc = 0;
		SC8915_PSTOP_Off();
	}

	if(count_acc > 5)
	{
//		OLED_display_string(4, 54, "CHAR");
		OLED_W_Icon11x8(2,0,50); // display 'on icon' .
	}
	else
	{
//		OLED_display_string(4, 54, "Wait");
		OLED_W_Icon11x8(0,0,50); // display 'off icon'.
//		OLED_Screen_Clean(4, 54, 24);
//		OLED_Set_Address(4, 54);
//		OLED_W_Num6x8(count_acc);
	}

	HAL_Delay(1000);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint16_t ccr_test = 0;

	if(htim->Instance == TIM14) // 10ms interrupt
	{
		if (ccr_test == 0)
		{
			ccr += 10;
			if(ccr > TIM14->ARR * 1 /3 )
			{
//				ccr = 0;
				ccr_test = 1;
			}
		}
		if(ccr_test == 1)
		{
			ccr -= 10;
			if(ccr < 20 )
			{
				ccr = 0;
				ccr_test = 0;
			}
		}

		if (isSwitchFullMode() )
		{
			if(isACC1on())
			{
				ccr = TIM14->ARR * 1 /3 ;
			}
			else
			{
// <- version 1.11
//				if(gCount_SCREEN_IDLE_TIME >= DELAY_SCREEN_IDLE_TIME)
//				{
//					ccr = 0;
//					ccr_test = 0;
//				}
			}
		}
//		else //if (isSwitchHalfMode() )
//		{
//			if(isACC2on())
//				;
//			else
//			{
//				if(gCount_SCREEN_IDLE_TIME >= DELAY_SCREEN_IDLE_TIME)
//				{
//					ccr = 0;
//					ccr_test = 0;
//				}
//			}
//		}
		else
		{
			ccr = 0;
			ccr_test = 0;
		}



		TIM14->CCR1 = ccr;
		TIM16->CCR1 = ccr;

		HAL_ADC_Start_DMA(&hadc, &adcval[0], 8);

	}


	if(htim->Instance == TIM6) // 1s interrupt
	{
//		HAL_ADC_Start_DMA(&hadc, &adcval[0], 8);
		gCount_SCREEN_IDLE_TIME++;
		if(gCount_SCREEN_IDLE_TIME >= DELAY_SCREEN_IDLE_TIME)
			gCount_SCREEN_IDLE_TIME = DELAY_SCREEN_IDLE_TIME;


		if( is_IN_EN_ON() )
//			if( is_IN_EN_ON() && isSwitchFullMode() )
		{
			gCount_FullMode_setting_Time++;
			if(gCount_FullMode_setting_Time >= FULLMODE_ACTION_TEME)
				gCount_FullMode_setting_Time = FULLMODE_ACTION_TEME;

		}
		else
		{
			gCount_FullMode_setting_Time = 0;
		}
	}



//	if(htim->Instance == TIM16)
//	{
//		if(ccr_test == 1)
//		{
//			ccr -= 500;
//			TIM14->CCR1 = ccr;
//			if(ccr <= 500)
//			{
//	//			ccr = 0;
//				ccr_test = 0;
//			}
//		}
//	}



}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{

//	InPut_voltage_Enable(adcval[0],adcval[5]);
//	SET_INPUT_Enable();
	SET_INPUT_Enable2();
//	if(hadc->Instance == hadc.Instance)
//	{
//		;
//	}
}





/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
	TIM16->CCR1 = TIM16->ARR / 4;

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
