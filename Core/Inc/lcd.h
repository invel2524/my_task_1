#ifndef __LCD_H__
#define __LCD_H__

#include "main.h"

// Define pins and ports
#define LCD_RS_Pin      GPIO_PIN_4
#define LCD_RS_Port     GPIOA

#define LCD_RW_Pin      GPIO_PIN_6
#define LCD_RW_Port     GPIOA

#define LCD_EN_Pin      GPIO_PIN_1
#define LCD_EN_Port     GPIOA

#define LCD_D4_Pin      GPIO_PIN_0
#define LCD_D4_Port     GPIOC

#define LCD_D5_Pin      GPIO_PIN_1
#define LCD_D5_Port     GPIOC

#define LCD_D6_Pin      GPIO_PIN_2
#define LCD_D6_Port     GPIOC

#define LCD_D7_Pin      GPIO_PIN_3
#define LCD_D7_Port     GPIOC

void LCD_Init(void);
void LCD_Send_Cmd(uint8_t);
void LCD_Send_Data(uint8_t);
void LCD_Send_String(char*);
void LCD_Set_Cursor(uint8_t row, uint8_t col);
void LCD_Clear(void);

#endif
