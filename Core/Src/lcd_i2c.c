#include "lcd_i2c.h"

extern I2C_HandleTypeDef hi2c1;
extern void MX_I2C1_Init(void);
extern void delay_us(uint32_t us);

static I2C_HandleTypeDef *lcd_hi2c;
static volatile uint8_t recuperation_en_cours = 0;

static uint8_t I2C_TxAvecRetry(uint8_t *data)
{
    if (HAL_I2C_Master_Transmit(lcd_hi2c, LCD_I2C_ADDR, data, 1, 15) == HAL_OK) return 1;
    delay_us(200);
    return (HAL_I2C_Master_Transmit(lcd_hi2c, LCD_I2C_ADDR, data, 1, 15) == HAL_OK);
}

static uint8_t LCD_WriteNibble(uint8_t nibble, uint8_t rs)
{
    uint8_t data_en = (nibble & 0xF0) | (rs ? 0x01 : 0x00) | 0x08 | 0x04;
    if (!I2C_TxAvecRetry(&data_en)) return 0;
    uint8_t data_dis = data_en & ~0x04;
    if (!I2C_TxAvecRetry(&data_dis)) return 0;
    return 1;
}

static uint8_t LCD_SendByte(uint8_t byte, uint8_t rs)
{
    if (!LCD_WriteNibble(byte & 0xF0, rs)) return 0;
    return LCD_WriteNibble((byte << 4) & 0xF0, rs);
}

static uint8_t LCD_SendCommand(uint8_t cmd) { return LCD_SendByte(cmd, 0); }
static uint8_t LCD_SendData(uint8_t data)   { return LCD_SendByte(data, 1); }

// --- Récupération de bus I2C : force le PCF8574 à relâcher SDA s'il est resté bloqué ---
static void I2C1_Bus_Recovery(void)
{
    __HAL_RCC_I2C1_FORCE_RESET();
    __HAL_RCC_I2C1_RELEASE_RESET();
    __HAL_RCC_I2C1_CLK_DISABLE();

    GPIO_InitTypeDef gi = {0};
    gi.Mode  = GPIO_MODE_OUTPUT_OD;
    gi.Pull  = GPIO_PULLUP;
    gi.Speed = GPIO_SPEED_FREQ_LOW;
    gi.Pin = GPIO_PIN_6; HAL_GPIO_Init(GPIOB, &gi);   // PB6 = SCL
    gi.Pin = GPIO_PIN_9; HAL_GPIO_Init(GPIOB, &gi);   // PB9 = SDA

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    delay_us(10);

    for (int i = 0; i < 9; i++)
    {
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET) break;  // SDA déjà libérée
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
        delay_us(5);
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
        delay_us(5);
    }

    // Condition STOP manuelle
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    delay_us(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    delay_us(5);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    delay_us(5);

    gi.Mode      = GPIO_MODE_AF_OD;
    gi.Alternate = GPIO_AF4_I2C1;
    gi.Pin = GPIO_PIN_6; HAL_GPIO_Init(GPIOB, &gi);
    gi.Pin = GPIO_PIN_9; HAL_GPIO_Init(GPIOB, &gi);

    __HAL_RCC_I2C1_CLK_ENABLE();
    MX_I2C1_Init();
}

void LCD_Init(I2C_HandleTypeDef *hi2c)
{
    lcd_hi2c = hi2c;
    HAL_Delay(50);
    LCD_WriteNibble(0x30, 0); HAL_Delay(5);
    LCD_WriteNibble(0x30, 0); HAL_Delay(1);
    LCD_WriteNibble(0x30, 0); HAL_Delay(1);
    LCD_WriteNibble(0x20, 0);
    LCD_SendCommand(0x28);
    LCD_SendCommand(0x0C);
    LCD_SendCommand(0x06);
    LCD_Clear();
}

void LCD_Clear(void)
{
    LCD_SendCommand(0x01);
    HAL_Delay(2);
}

void LCD_SetCursor(uint8_t row, uint8_t col)
{
    static const uint8_t offsets[4] = {0x00, 0x40, 0x14, 0x54};
    if (row > 3) row = 3;
    LCD_SendCommand(0x80 | (offsets[row] + col));
}

uint8_t LCD_SendString(char *str)
{
    uint8_t ok = 1;
    while (*str) { if (!LCD_SendData((uint8_t)*str++)) { ok = 0; break; } }
    if (!ok) {
        recuperation_en_cours = 1;
        I2C1_Bus_Recovery();
        LCD_Init(lcd_hi2c);
        recuperation_en_cours = 0;
    }
    return ok;
}

uint8_t LCD_Is_Recovering(void) { return recuperation_en_cours; }
