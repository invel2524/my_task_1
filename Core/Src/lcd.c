#include "lcd.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "string.h"

void LCD_Enable(void) {
    HAL_GPIO_WritePin(LCD_EN_Port, LCD_EN_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD_EN_Port, LCD_EN_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
}

void LCD_Send_Nibble(uint8_t nibble) {
    HAL_GPIO_WritePin(LCD_D4_Port, LCD_D4_Pin, (nibble >> 0) & 0x01);
    HAL_GPIO_WritePin(LCD_D5_Port, LCD_D5_Pin, (nibble >> 1) & 0x01);
    HAL_GPIO_WritePin(LCD_D6_Port, LCD_D6_Pin, (nibble >> 2) & 0x01);
    HAL_GPIO_WritePin(LCD_D7_Port, LCD_D7_Pin, (nibble >> 3) & 0x01);
    LCD_Enable();
}

void LCD_Send_Cmd(uint8_t cmd) {
    HAL_GPIO_WritePin(LCD_RS_Port, LCD_RS_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(LCD_RW_Port, LCD_RW_Pin, GPIO_PIN_RESET);

    LCD_Send_Nibble(cmd >> 4);
    LCD_Send_Nibble(cmd & 0x0F);

    HAL_Delay(2);
}

void LCD_Send_Data(uint8_t data) {
    HAL_GPIO_WritePin(LCD_RS_Port, LCD_RS_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(LCD_RW_Port, LCD_RW_Pin, GPIO_PIN_RESET);

    LCD_Send_Nibble(data >> 4);
    LCD_Send_Nibble(data & 0x0F);

    HAL_Delay(2);
}

void LCD_Init(void) {
    HAL_Delay(50);  // Wait for LCD to power up

    LCD_Send_Nibble(0x03); HAL_Delay(5);
    LCD_Send_Nibble(0x03); HAL_Delay(5);
    LCD_Send_Nibble(0x03); HAL_Delay(1);
    LCD_Send_Nibble(0x02);  // 4-bit mode

    LCD_Send_Cmd(0x28); // 4-bit, 2-line, 5x8 font
    LCD_Send_Cmd(0x0C); // Display ON, cursor OFF
    LCD_Send_Cmd(0x06); // Entry mode
    LCD_Send_Cmd(0x01); // Clear
    HAL_Delay(2);
}

void LCD_Send_String(char *str) {
    while (*str) {
        LCD_Send_Data(*str++);
    }
}

void LCD_Set_Cursor(uint8_t row, uint8_t col) {
    uint8_t pos = (row == 0) ? 0x00 : 0x40;
    LCD_Send_Cmd(0x80 | (pos + col));
}

void LCD_Clear(void) {
    LCD_Send_Cmd(0x01);
    HAL_Delay(2);
}
