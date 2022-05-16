/*
 * SC8915.C
 *
 *  Created on: 2020. 12. 22.
 *      Author: noto_GTX55
 */

/* Includes ------------------------------------------------------------------*/
#include "SC8915.h"
#include "i2c.h"
#include "OLED_SSD1306.h"

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SC8915_Address 	0xE8 /*slave addresses write*/
#define ACC1_LOW_VOLTAGE 1400
#define ACC2_LOW_VOLTAGE 1400
#define BAT_LOW_VOLTAGE 1000

#define TEMP30_CAL_ADDR ((uint16_t*) ((uint32_t)0x1FFFF7B8))
#define VREFINT_CAL		((uint16_t*) ((uint32_t)0x1FFFF7BA))
#define VDD_CALIB		((uint32_t) (3300))
#define VDD_APPLI		((uint32_t) (3000))
#define AVG_SLOPE		((uint32_t) (5336))

/* USER CODE END PD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#if 0
typedef struct
{
  uint8_t VBAT_SET;			/*!<			Address offset: 0x00 */
  uint8_t VBUSREF_I_SET;	/*!<			Address offset: 0x01 */
  uint8_t VBUSREF_I_SET2;	/*!< 			Address offset: 0x02 */
  uint8_t VBUSREF_E_SET;	/*!<			Address offset: 0x03 */
  uint8_t VBUSFEF_E_SET2;	/*!<			Address offset: 0x04 */
  uint8_t IBUS_LIM_SET;		/*!<			Address offset: 0x05 */
  uint8_t IBAT_LIM_SET;		/*!<			Address offset: 0x06 */
  uint8_t ViNREG_SET;		/*!< 			Address offset: 0x07 */
  uint8_t RATIO;			/*!<			Address offset: 0x08 */
  uint8_t CTRL0_SET;		/*!<			Address offset: 0x09 */
  uint8_t CTRL1_SET;		/*!<			Address offset: 0x0A */
  uint8_t CTRL2_SET;		/*!<			Address offset: 0x0B */
  uint8_t CTRL3_SET;		/*!< 			Address offset: 0x0C */
  uint8_t VBUS_FB_VALUE;	/*!<			Address offset: 0x0D */
  uint8_t VBUS_FB_VALUE2;	/*!<			Address offset: 0x0E */
  uint8_t VBAT_FB_VALUE;	/*!<			Address offset: 0x0F */
  uint8_t VBAT_FB_VALUE2;	/*!<			Address offset: 0x10 */
  uint8_t IBUS_VALUE;		/*!< 			Address offset: 0x11 */
  uint8_t IBUS_VALUE2;		/*!<			Address offset: 0x12 */
  uint8_t IBAT_VALUE;		/*!<			Address offset: 0x13 */
  uint8_t IBAT_VALUE2;		/*!<			Address offset: 0x14 */
  uint8_t ADIN_VALUE;		/*!< 			Address offset: 0x15 */
  uint8_t ADIN_VALUE2;		/*!<			Address offset: 0x16 */
  uint8_t STATUS;			/*!<			Address offset: 0x17 */
  uint8_t RESERBED0;		/*!<			Address offset: 0x18 */
  uint8_t MASK;				/*!<			Address offset: 0x19 */
  uint8_t RESERBED1;		/*!< 			Address offset: 0x1A */
  uint8_t RESERBED2;		/*!<			Address offset: 0x1B */
} SC8915_REGISTER_TypeDef;
#endif


extern volatile uint16_t adcval[8];
uint8_t delay_OUT_EN = 0;
/* USER CODE END PD */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */



/* USER CODE END PFP */

void SC8915_init(void)
{
//	SC8915_W_Command(0x00,0x11);	// VBAT_SET
	SC8915_W_Command(0x00,0x21);	// VBAT_SET

//	SC8915_W_Command(0x01,0x31);	// VBUSREF_I_SET
//	SC8915_W_Command(0x02,0xC0);	// VBUSREF_I_SET2

//	SC8915_W_Command(0x03,0x7C);	// VBUSREF_E_SET
//	SC8915_W_Command(0x04,0xC0);	// VBUSREF_E_SET2

	SC8915_W_Command(0x05,0x19);	// IBUS_LIM_SET
//	SC8915_W_Command(0x05,0xC0);	// IBUS_LIM_SET

	SC8915_W_Command(0x06,0x20);	// IBAT_LIM_SET
//	SC8915_W_Command(0x06,0x6A);	// IBAT_LIM_SET

	SC8915_W_Command(0x07,0x72);	// VINREG_SET

	SC8915_W_Command(0x08,0x38);	// RATIO

	SC8915_W_Command(0x09,0x04); 	// CTRL0_SET
	SC8915_W_Command(0x0A,0xA5); 	// CTRL1_SET
	SC8915_W_Command(0x0B,0x01); 	// CTRL2_SET
	SC8915_W_Command(0x0C,0x22); 	// CTRL3_SET

	SC8915_W_Command(0x19,0x80);	// MASK
}

void SC8915_init_Default(void)
{
	SC8915_W_Command(0x00,0x01);	// VBAT_SET

	SC8915_W_Command(0x01,0x31);	// VBUSREF_I_SET
	SC8915_W_Command(0x02,0xC0);	// VBUSREF_I_SET2

	SC8915_W_Command(0x03,0x7C);	// VBUSREF_E_SET
	SC8915_W_Command(0x04,0xC0);	// VBUSREF_E_SET2

	SC8915_W_Command(0x05,0x19);	// IBUS_LIM_SET
	SC8915_W_Command(0x06,0x20);	// IBAT_LIM_SET
	SC8915_W_Command(0x07,0x2C);	// VINREG_SET

	SC8915_W_Command(0x08,0x38);	// RATIO

	SC8915_W_Command(0x09,0x04);    // CTRL0_SET
	SC8915_W_Command(0x0A,0x01);    // CTRL1_SET
	SC8915_W_Command(0x0B,0x01); 	// CTRL2_SET
	SC8915_W_Command(0x0C,0x02); 	// CTRL3_SET

#if 0 // READ ONLY
	SC8915_W_Command(0x0D,0x00);	// VBUS_FB_VALUE
	SC8915_W_Command(0x0E,0x00);	// VBUS_FB_VALUE2

	SC8915_W_Command(0x0F,0x00);	// VBAT_FB_VALUE
	SC8915_W_Command(0x10,0x00);	// VBAT_FB_VALUE2

	SC8915_W_Command(0x11,0x00);	// IBUS_VALUE
	SC8915_W_Command(0x12,0x00);	// IBUS_VALUE2

	SC8915_W_Command(0x13,0x00);	// IBAT_VALUE
	SC8915_W_Command(0x14,0x00);	// IBAT_VALUE2

	SC8915_W_Command(0x15,0x00);	// ADIN_VALUE
	SC8915_W_Command(0x16,0x00);	// ADIN_VALUE_2

	SC8915_W_Command(0x17,0x00);	// STATUS
#endif

	SC8915_W_Command(0x19,0x80);	// MASK
}

void SC8915_FULL_mode(void)
{
//	SC8915_W_Command(0x00,0x11);	// VBAT_SET
	SC8915_W_Command(0x00,0x21);	// VBAT_SET

//	SC8915_W_Command(0x01,0x31);	// VBUSREF_I_SET
//	SC8915_W_Command(0x02,0xC0);	// VBUSREF_I_SET2

//	SC8915_W_Command(0x03,0x7C);	// VBUSREF_E_SET
//	SC8915_W_Command(0x04,0xC0);	// VBUSREF_E_SET2

//	SC8915_W_Command(0x05,0x19);	// IBUS_LIM_SET
	SC8915_W_Command(0x05,0xB4);	// IBUS_LIM_SET (D7 -> D2 ) 2021.12.08 v2.03

//	SC8915_W_Command(0x06,0x20);	// IBAT_LIM_SET
	SC8915_W_Command(0x06,0x4B);	// IBAT_LIM_SET (57 -> 55 ) 2021.12.08 v2.03


	SC8915_W_Command(0x07,0x72);	// VINREG_SET  // 2021.12.08 ( 72->70 ) cutoff voltage

	SC8915_W_Command(0x08,0x38);	// RATIO

	SC8915_W_Command(0x09,0x05); 	// CTRL0_SET // modyfied by 2021.10.13  0x04-> 0x05
//	SC8915_W_Command(0x09,0x07); 	// CTRL0_SET // modyfied by 2021.0422
	SC8915_W_Command(0x0A,0xE5); 	// CTRL1_SET
	SC8915_W_Command(0x0B,0x01); 	// CTRL2_SET
	SC8915_W_Command(0x0C,0x22); 	// CTRL3_SET

	SC8915_W_Command(0x19,0x80);	// MASK

//	OLED_display_string(0, 54, "FULL");
//	OLED_Screen_Clean(2, 54, 24);
	OLED_W_Icon11x8(4,2,50); // full mode display 4'empty.
}

void SC8915_HALF_mode(void)
{
//	SC8915_W_Command(0x00,0x11);	// VBAT_SET
	SC8915_W_Command(0x00,0x21);	// VBAT_SET

//	SC8915_W_Command(0x01,0x31);	// VBUSREF_I_SET
//	SC8915_W_Command(0x02,0xC0);	// VBUSREF_I_SET2

//	SC8915_W_Command(0x03,0x7C);	// VBUSREF_E_SET
//	SC8915_W_Command(0x04,0xC0);	// VBUSREF_E_SET2

//	SC8915_W_Command(0x05,0x19);	// IBUS_LIM_SET
	SC8915_W_Command(0x05,0x5A);	// IBUS_LIM_SET

//	SC8915_W_Command(0x06,0x20);	// IBAT_LIM_SET
	SC8915_W_Command(0x06,0x25);	// IBAT_LIM_SET

	SC8915_W_Command(0x07,0x72);	// VINREG_SET // 2021.12.08 ( 72->70 ) cutoff voltage

	SC8915_W_Command(0x08,0x38);	// RATIO

	SC8915_W_Command(0x09,0x05); 	// CTRL0_SET // modyfied by 2021.10.13 0x04 -> 0x05
//	SC8915_W_Command(0x09,0x07); 	// CTRL0_SET // modyfied by 2021.0422
	SC8915_W_Command(0x0A,0xE5); 	// CTRL1_SET
	SC8915_W_Command(0x0B,0x01); 	// CTRL2_SET
	SC8915_W_Command(0x0C,0x22); 	// CTRL3_SET

	SC8915_W_Command(0x19,0x80);	// MASK

//	OLED_display_string(2, 54, "HALF");
//	OLED_Screen_Clean(0, 54, 24);
	OLED_W_Icon11x8(3,2,50); // half mode display 3'lowcharge.

}

void SC8915_TEST_7A_mode(void)
{
//	SC8915_W_Command(0x00,0x11);	// VBAT_SET
	SC8915_W_Command(0x00,0x21);	// VBAT_SET

//	SC8915_W_Command(0x01,0x31);	// VBUSREF_I_SET
//	SC8915_W_Command(0x02,0xC0);	// VBUSREF_I_SET2

//	SC8915_W_Command(0x03,0x7C);	// VBUSREF_E_SET
//	SC8915_W_Command(0x04,0xC0);	// VBUSREF_E_SET2

//	SC8915_W_Command(0x05,0x19);	// IBUS_LIM_SET
	SC8915_W_Command(0x05,0xB4);	// IBUS_LIM_SET

//	SC8915_W_Command(0x06,0x20);	// IBAT_LIM_SET
	SC8915_W_Command(0x06,0x48);	// IBAT_LIM_SET

	SC8915_W_Command(0x07,0x72);	// VINREG_SET

	SC8915_W_Command(0x08,0x38);	// RATIO

	SC8915_W_Command(0x09,0x04); 	// CTRL0_SET
	SC8915_W_Command(0x0A,0xE5); 	// CTRL1_SET
	SC8915_W_Command(0x0B,0x01); 	// CTRL2_SET
	SC8915_W_Command(0x0C,0x22); 	// CTRL3_SET

	SC8915_W_Command(0x19,0x80);	// MASK
}



void SC8915_W_Command(uint8_t cmd, uint8_t data)
{
	uint8_t buff[2];
	buff[0] = cmd;
	buff[1] = data;

	if(HAL_I2C_Master_Transmit(&hi2c1, SC8915_Address, &buff[0], 2, 100) != HAL_OK)
	{
		Error_Handler();
	}
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

}


void SC8915_R_DATA(uint8_t reg, uint8_t* data_buffer, uint16_t buffer_size)
{
	//	if(HAL_I2C_Master_Receive(&hi2c1, SC8915_Address, data_buffer, buffer_size, 100)!= HAL_OK)
	if(HAL_I2C_Mem_Read(&hi2c1, SC8915_Address,(uint16_t)reg, I2C_MEMADD_SIZE_8BIT, data_buffer, buffer_size, 100)!= HAL_OK)
	{
		Error_Handler();
	}
	while(HAL_I2C_GetState(&hi2c1) != HAL_I2C_STATE_READY);

}



void display_TEST(uint8_t* ADC_VALUE)
{

}

void display_DATA(int data)
{

}


void SC8915_PSTOP_Off(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET); // PSTOP is off.
}

void SC8915_PSTOP_On(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET); // PSTOP is On.
}

void IN_EN1_voltage_Off(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // IN_EN is off.
}

void IN_EN1_voltage_On(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // IN_EN is on.
}

void IN_EN2_voltage_Off(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // IN_EN2 is off.
}

void IN_EN2_voltage_On(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // IN_EN2 is on.
}

void OUT_EN_voltage_Off(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}

void OUT_EN_voltage_On(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
}

void InPut_voltage_Enable(uint16_t acc_val1, uint16_t acc_val2)
{
	int limit_voltage = 1000;
	static int nCount_acc_on = 0;

	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
	{
		if( acc_val1 > limit_voltage )
		{
			nCount_acc_on++;
			if( nCount_acc_on > 10 )
			{
				if(acc_val2 > limit_voltage)
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // EN_EN is On.
		//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // EN_EN is OFF.
		//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // EN_EN2 is On.
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // EN_EN2 is OFF.
				}
				else
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // EN_EN is On.
		//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // EN_EN is OFF.
		//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // EN_EN2 is On.
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // EN_EN2 is OFF.
				}
				nCount_acc_on = 10;
			}
		}
		else
		{
			nCount_acc_on = 0;
			if(acc_val2 > limit_voltage)
			{
	//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // EN_EN is On.
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // EN_EN is OFF.
	//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // EN_EN2 is On.
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // EN_EN2 is OFF.
			}
			else
			{
	//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // EN_EN is On.
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // EN_EN is OFF.
	//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // EN_EN2 is On.
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // EN_EN2 is OFF.
			}
		}
	}
	else if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_RESET)
	{
		if( acc_val1 > limit_voltage )
		{
			nCount_acc_on++;
			if(nCount_acc_on > 10)
			{
				if(acc_val2 > limit_voltage)
				{
		//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // EN_EN is On.
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // EN_EN is OFF.
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // EN_EN2 is On.
		//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // EN_EN2 is OFF.
				}
				else
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // EN_EN is On.
		//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // EN_EN is OFF.
		//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // EN_EN2 is On.
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // EN_EN2 is OFF.
				}
				nCount_acc_on = 10;
			}
		}
		else
		{
			if(acc_val2 > limit_voltage)
			{
				nCount_acc_on++;
				if(nCount_acc_on > 10)
				{
		//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // EN_EN is On.
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // EN_EN is OFF.
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // EN_EN2 is On.
		//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // EN_EN2 is OFF.
					nCount_acc_on = 10;
				}
			}
			else
			{
				nCount_acc_on = 0;
	//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // EN_EN is On.
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // EN_EN is OFF.
	//			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // EN_EN2 is On.
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // EN_EN2 is OFF.
			}
		}
	}
	else
	{
		nCount_acc_on  = 0;
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // EN_EN is On.
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // EN_EN is OFF.
//		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // EN_EN2 is On.
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // EN_EN2 is OFF.

	}

}

int VBUS_mV_converter(uint8_t VBUS_FB_value,uint8_t VBUS_FB_value2 )
{
	// VBUS_ratio is 12.5
	return VBUS_FB_value*100 + (VBUS_FB_value2>>6)*25 + 25;
}

int VBAT_mV_converter(uint8_t VBAT_FB_value,uint8_t VBAT_FB_value2)
{
	// VBAT_moniter_ratio is 12.5
	return VBAT_FB_value*100 + (VBAT_FB_value2>>6)*25 + 25;
}

int IBUS_mA_converter(uint8_t IBUS_value,uint8_t IBUS_value2 )
{
	// ((4*IBUS_value+(IBUS_value2>>6)+1)*2*IBUS_RATIO*10m/RS1) /1200
	// RS1 = 3m // IBUS_RATIO = 3
	return (IBUS_value*4+(IBUS_value2>>6)+1)*50/3;
}

int IBAT_mA_converter(uint8_t IBAT_value,uint8_t IBAT_value2 )
{
	// ((4*IBAT_value+(IBAT_value2>>6)+1)*2*IBAT_RATIO*10m/RS2) /1200
	// IBAT_RATIO = 12 // RS2 = 5m
	return (IBAT_value*4+(IBAT_value2>>6)+1)*40;
}

int isSwitchFullMode(void)
{
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) == GPIO_PIN_RESET)
		return 1;
	else
		return 0;
}

int isSwitchHalfMode(void)
{
	if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) == GPIO_PIN_RESET)
		return 1;
	else
		return 0;
}

void Set_IN_EN_ON(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET); // IN_EN is On.
}

void Set_IN_EN_OFF(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET); // IN_EN is OFF.
}

int is_IN_EN_ON(void)
{
	return ( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_15) == GPIO_PIN_SET );
}



void Set_IN_EN2_ON(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET); // IN_EN2 is On.
}

void Set_IN_EN2_OFF(void)
{
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET); // IN_EN2 is OFF.
}

int is_IN_EN2_ON(void)
{
	return ( HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_14) == GPIO_PIN_SET );
}



void Set_OUT_EN_ON(void)
{
	if(delay_OUT_EN < 2)
	{
		delay_OUT_EN ++;
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	}
}

void Set_OUT_EN_OFF(void)
{
	if(delay_OUT_EN == 0)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	}
	else
		delay_OUT_EN--;
}


int isACC1on(void)
{
//	static ACC1on_state=0;

	if(adcval[0]>ACC1_LOW_VOLTAGE)
		return 1;
	else
		return 0;
}

int isACC2on(void)
{
	if(adcval[5]>ACC2_LOW_VOLTAGE)
		return 1;
	else
		return 0;
}

int isBATon(void)
{
	if(adcval[4]>BAT_LOW_VOLTAGE)
		return 1;
	else
		return 0;
}

void SET_INPUT_Enable(void)
{
	static int nCount_acc_on = 0;

	if(isSwitchFullMode())
	{
		if( isACC1on() )
		{
			nCount_acc_on++;
			if( nCount_acc_on > 10 )
			{
				Set_IN_EN_ON();
				Set_IN_EN2_OFF();
				nCount_acc_on = 10;
			}
		}
		else
		{
			nCount_acc_on = 0;
			Set_IN_EN_OFF();
			Set_IN_EN2_OFF();
		}
	}
	else// if(isSwitchHalfMode())
	{
		if(isACC2on())
		{
			nCount_acc_on++;
			if(nCount_acc_on > 10)
			{
				Set_IN_EN_OFF();
				Set_IN_EN2_ON();
				nCount_acc_on = 10;
			}
		}
		else
		{
			nCount_acc_on = 0;
			Set_IN_EN_OFF();
			Set_IN_EN2_OFF();
		}
	}
//	else
//	{
//		nCount_acc_on  = 0;
//		Set_IN_EN_OFF();
//		Set_IN_EN2_OFF();
//
//	}
}

int mcuADCto_mV(uint16_t adcValue)
{
	if(adcValue < 10)
		return 0;
	else
		return ((int)adcValue*100+9255)/1259;
}

int mcuTemperature(uint16_t adcValue)
{
	int32_t temperature;
//	temperature = ((uint32_t) *TEMP30_CAL_ADDR - ((uint32_t) adcValue * VDD_APPLI / VDD_CALIB)) * 1000;
	temperature = ((int32_t) *TEMP30_CAL_ADDR - (((int32_t) adcval[6] * (int32_t) adcval[7] ) / (int32_t) *VREFINT_CAL ) )  * 1000;
	temperature = (temperature /(int32_t) AVG_SLOPE) ;//+ 30;
//	temperature = ((int32_t) *TEMP30_CAL_ADDR - (((int32_t) adcval[6] * 3530 ) / 3300 ) )  * 1000;
//	temperature = (temperature /(int32_t) AVG_SLOPE) + 30;





//	temperature = (int32_t) *TEMP30_CAL_ADDR * 555 - (int32_t) adcValue * 505 - 5932 ;
//	temperature = (temperature / (int32_t)AVG_SLOPE);

//		temperature = ((int32_t) *TEMP30_CAL_ADDR + 49 - (int32_t) adcValue ) * 1000 ;
//		temperature = ( temperature / 5586 );

//	temperature = (1847 - (int32_t) adcValue ) * 1000 ;
//	temperature = ( temperature / 5586 );

	return temperature;
}

int mcuTemperature2(uint16_t adcValue)
{
	int32_t temperature;
//	temperature = ((uint32_t) *TEMP30_CAL_ADDR - ((uint32_t) adcValue * VDD_APPLI / VDD_CALIB)) * 1000;
///	temperature = ((int32_t) *TEMP30_CAL_ADDR - (((int32_t) adcval[6] * (int32_t) adcval[7] ) / (int32_t) *VREFINT_CAL ) )  * 1000;
//	temperature = (temperature /(int32_t) AVG_SLOPE) ;//+ 30;

//	temperature = (int32_t) *TEMP30_CAL_ADDR * 555 - (int32_t) adcValue * 505 - 5932 ;
//	temperature = (temperature / (int32_t)AVG_SLOPE);

	//	temperature = ((int32_t) *TEMP30_CAL_ADDR + 49 - (int32_t) adcValue ) * 1000 ;
	//	temperature = ( temperature / 5586 );

	temperature = (1847 - (int32_t) adcValue ) * 1000 ;
	temperature = ( temperature / 5586 );

	return temperature;
}


void SC8915_work_per_Second(void)
{

}

void SET_INPUT_Enable2(void)
{
//	static int nCount_acc_on = 0;

	int Vacc1, Vacc2;

	int ACC_high = 112, ACC_LOW = 106; //12v

	Vacc1 = mcuADCto_mV(adcval[0]);
	Vacc2 = mcuADCto_mV(adcval[5]);

	if(1)
//		if(isSwitchFullMode())
	{
		if ( is_IN_EN_ON() ) // IN_EN is On.)
		{
			if( Vacc1 > ACC_LOW )
			{
				Set_IN_EN_ON();
				Set_IN_EN2_OFF();
			}
			else
			{
				Set_IN_EN_OFF();
				Set_IN_EN2_OFF();
			}
		}
		else
		{
			if( Vacc1 >= ACC_high )
			{
				Set_IN_EN_ON();
				Set_IN_EN2_OFF();
			}
			else
			{
				Set_IN_EN_OFF();
				Set_IN_EN2_OFF();
			}
		}
	}
	else// if(isSwitchHalfMode())
	{
		if( is_IN_EN2_ON() )
		{
			if(Vacc2 > ACC_LOW )
			{
				Set_IN_EN_OFF();
				Set_IN_EN2_ON();
			}
			else
			{
				Set_IN_EN_OFF();
				Set_IN_EN2_OFF();
			}
		}
		else
		{
			if(Vacc2 >= ACC_high )
			{
				Set_IN_EN_OFF();
				Set_IN_EN2_ON();
			}
			else
			{
				Set_IN_EN_OFF();
				Set_IN_EN2_OFF();
			}
		}
	}
//	else
//	{
//		Set_IN_EN_OFF();
//		Set_IN_EN2_OFF();
//	}
}

