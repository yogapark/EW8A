/*
 * OLED_SSD1306.h
 *
 *  Created on: 2020. 11. 27.
 *      Author: noto_GTX55
 */

#ifndef INC_OLED_SSD1306_H_
#define INC_OLED_SSD1306_H_

/* Includes ------------------------------------------------------------------*/
#include "main.h"


void OLED_Sellect_first(void);

void OLED_Sellect_second(void);


void OLED_W_Command(uint8_t cmd);
void OLED_Set_Address(uint8_t page, uint8_t column);
void OLED_W_DATA(uint8_t* data_buffer, uint16_t buffer_size);
void OLED_Init(void);
void OLED_Page_Clear(uint8_t Clear_byte);
void OLED_Screen_Clear(void);
void OLED_Screen_Clean(uint8_t page, uint8_t column, int data);

void OLED_image_test(uint8_t page);
void OLED_image_test_ener(void);
void OLED_image_test_iROAD(void);
void OLED_image_iROAD(void);
void OLED_image_Loading(void);

void OLED_W_Num6x8(uint8_t num);
void OLED_W_BChar6x8(uint8_t character);
void OLED_W_sChar6x8(uint8_t character);

void OLED_W_Char(uint8_t Character, uint8_t page, uint8_t column);

void OLED_W_Num11x16(uint8_t num, uint8_t page, uint8_t column);
void OLED_W_Icon11x8(uint8_t num, uint8_t page, uint8_t column);

void OLED_display_mV(uint8_t page, uint8_t column, int data);
void OLED_display_mA(uint8_t page, uint8_t column, int data);
void OLED_display_string(uint8_t page, uint8_t column, char *string);
void OLED_display_string_100m(uint8_t page, uint8_t column, char *string);
void OLED_display_string_Tem(uint8_t page, uint8_t column, char *string);

void OLED_display_V_Num11x16(uint8_t page, uint8_t column,int num);
void OLED_display_A(uint8_t page, uint8_t column, int data);

void OLED_display_Battery_Icon(uint8_t page, uint8_t column, int mVBatValue);
void OLED_display_Battery_Icon_wide(uint8_t page, uint8_t column, int mVBatValue);
void OLED_display_Version(uint8_t page, uint8_t column, int nVersion);
void OLED_display_live_action(uint8_t page, uint8_t column);

int OLED_IsScreenOn(void);
int OLED_IsScreenOff(void);
void OLED_SetScreenOn(void);
void OLED_SetScreenOff(void);
void OLED_SCREEN_ON(void);
void OLED_SCREEN_OFF(void);

#endif /* INC_OLED_SSD1306_H_ */
