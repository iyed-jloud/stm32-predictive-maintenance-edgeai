#ifndef LCD_I2C_H
#define LCD_I2C_H
#include "stm32f4xx_hal.h"
#define LCD_I2C_ADDR (0x27 << 1)
void LCD_Init(I2C_HandleTypeDef *hi2c);
void LCD_Clear(void);
void LCD_SetCursor(uint8_t row, uint8_t col);
uint8_t LCD_SendString(char *str);
uint8_t LCD_Is_Recovering(void);
#endif
