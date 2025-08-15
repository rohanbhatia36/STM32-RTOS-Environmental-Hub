#ifndef SH1106_H
#define SH1106_H

#include "main.h"
#include "fonts.h"

//current hardware configuration
#define SH1106_SPI_PORT     hspi2
extern SPI_HandleTypeDef    SH1106_SPI_PORT;

#define SH1106_CS_PORT      GPIOB
#define SH1106_CS_PIN       GPIO_PIN_6

#define SH1106_DC_PORT      GPIOC
#define SH1106_DC_PIN       GPIO_PIN_7

#define SH1106_RES_PORT     GPIOA
#define SH1106_RES_PIN      GPIO_PIN_9

//display properties
#define SH1106_WIDTH        128
#define SH1106_HEIGHT       64

//color enumeration
typedef enum {
    Black = 0x00, //black color: no pixel
    White = 0x01  //white color: pixel is set
} SH1106_COLOR;

//functions
void sh1106_Init(void);
void sh1106_UpdateScreen(void);
void sh1106_Fill(SH1106_COLOR color);
void sh1106_DrawPixel(uint8_t x, uint8_t y, SH1106_COLOR color);
void sh1106_WriteString(char* str, FontDef font, SH1106_COLOR color);
void sh1106_SetCursor(uint8_t x, uint8_t y);
char sh1106_WriteChar(char ch, FontDef font, SH1106_COLOR color);

#endif //SH1106_H
