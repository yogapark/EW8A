/*
 * SC8915.h
 *
 *  Created on: 2020. 12. 22.
 *      Author: noto_GTX55
 */

#ifndef INC_SC8915_H_
#define INC_SC8915_H_

#include "main.h"

void SC8915_init(void);
void SC8915_init_Default(void);
void SC8915_FULL_mode(void);
void SC8915_HALF_mode(void);
void SC8915_TEST_7A_mode(void);

void SC8915_W_Command(uint8_t cmd , uint8_t data);
void SC8915_R_DATA(uint8_t reg, uint8_t* data_buffer, uint16_t buffer_size);

void SC8915_PSTOP_Off(void);
void SC8915_PSTOP_On(void);
void IN_EN1_voltage_Off(void);
void IN_EN1_voltage_On(void);
void IN_EN2_voltage_Off(void);
void IN_EN2_voltage_On(void);
void OUT_EN_voltage_Off(void);
void OUT_EN_voltage_On(void);

void InPut_voltage_Enable(uint16_t acc_val1, uint16_t acc_val2);

int VBUS_mV_converter(uint8_t VBUS_FB_value,uint8_t VBUS_FB_value2);
int VBAT_mV_converter(uint8_t VBAT_FB_value,uint8_t VBAT_FB_value2);
int IBUS_mA_converter(uint8_t IBUS_value,uint8_t IBUS_value2);
int IBAT_mA_converter(uint8_t IBAT_value,uint8_t IBAT_value2);

int isSwitchFullMode(void);
int isSwitchHalfMode(void);

void Set_IN_EN_ON(void);
void Set_IN_EN_OFF(void);
int is_IN_EN_ON(void);
void Set_IN_EN2_ON(void);
void Set_IN_EN2_OFF(void);
int is_IN_EN2_ON(void);

void Set_OUT_EN_ON(void);
void Set_OUT_EN_OFF(void);

int isACC1on(void);
int isACC2on(void);
int isBATon(void);

void SET_INPUT_Enable(void);
void SET_INPUT_Enable2(void);

int mcuADCto_mV(uint16_t adcValue);
int mcuTemperature(uint16_t adcValue);
int mcuTemperature3(void);

#endif /* INC_SC8915_H_ */
