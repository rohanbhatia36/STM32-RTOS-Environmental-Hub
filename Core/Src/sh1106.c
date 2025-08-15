#include "sh1106.h"
#include <string.h>

//screen buffer (each byte for 8 vertical pixels)
static uint8_t SH1106_Buffer[SH1106_WIDTH * SH1106_HEIGHT / 8];

//current cursor position
static uint8_t currentX = 0;
static uint8_t currentY = 0;

//internal functions
void sh1106_WriteCommand(uint8_t command);
void sh1106_WriteData(uint8_t* data, uint16_t size);

//initialization function
void sh1106_Init(void) {
    //hardware reset
    HAL_GPIO_WritePin(SH1106_RES_PORT, SH1106_RES_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(SH1106_RES_PORT, SH1106_RES_PIN, GPIO_PIN_SET);
    HAL_Delay(10);

    sh1106_WriteCommand(0xAE); //display OFF
    sh1106_WriteCommand(0x20); //set Memory Addressing Mode
    sh1106_WriteCommand(0x10); //00: Horizontal Addressing Mode; 01: Vertical Addressing Mode; 10: Page Addressing Mode (RESET); 11: Invalid
    sh1106_WriteCommand(0xB0); //set Page Start Address for Page Addressing Mode, 0-7
    sh1106_WriteCommand(0xC8); //set COM Output Scan Direction
    sh1106_WriteCommand(0x00); //set low column address
    sh1106_WriteCommand(0x10); //set high column address
    sh1106_WriteCommand(0x40); //set start line address
    sh1106_WriteCommand(0x81); //set contrast control register
    sh1106_WriteCommand(0xFF);
    sh1106_WriteCommand(0xA1); //set segment re-map 0 to 127
    sh1106_WriteCommand(0xA6); //set normal display
    sh1106_WriteCommand(0xA8); //set multiplex ratio (1 to 64)
    sh1106_WriteCommand(0x3F); //
    sh1106_WriteCommand(0xA4); //0xa4: Output follows RAM content; 0xa5: Output ignores RAM content
    sh1106_WriteCommand(0xD3); //set display offset
    sh1106_WriteCommand(0x00); //no offset
    sh1106_WriteCommand(0xD5); //set display clock divide ratio/oscillator frequency
    sh1106_WriteCommand(0xF0); //set divide ratio
    sh1106_WriteCommand(0xD9); //set pre-charge period
    sh1106_WriteCommand(0x22); //
    sh1106_WriteCommand(0xDA); //set com pins hardware configuration
    sh1106_WriteCommand(0x12);
    sh1106_WriteCommand(0xDB); //set vcomh
    sh1106_WriteCommand(0x20); //0x20: 0.77xVcc
    sh1106_WriteCommand(0x8D); //set DC-DC enable
    sh1106_WriteCommand(0x14); //
    sh1106_WriteCommand(0xAF); //turn on display

    sh1106_Fill(Black);
    sh1106_UpdateScreen();
}

//fills the screen buffer with a color
void sh1106_Fill(SH1106_COLOR color) {
    memset(SH1106_Buffer, (color == Black) ? 0x00 : 0xFF, sizeof(SH1106_Buffer));
}

//writes the screen buffer to the display
void sh1106_UpdateScreen(void) {
    for (uint8_t i = 0; i < 8; i++) {
        sh1106_WriteCommand(0xB0 + i);
        sh1106_WriteCommand(0x02);
        sh1106_WriteCommand(0x10);
        sh1106_WriteData(&SH1106_Buffer[SH1106_WIDTH * i], SH1106_WIDTH);
    }
}

// Draws a single pixel
void sh1106_DrawPixel(uint8_t x, uint8_t y, SH1106_COLOR color) {
    if (x >= SH1106_WIDTH || y >= SH1106_HEIGHT) {
        return;
    }
    if (color == White) {
        SH1106_Buffer[x + (y / 8) * SH1106_WIDTH] |= 1 << (y % 8);
    } else {
        SH1106_Buffer[x + (y / 8) * SH1106_WIDTH] &= ~(1 << (y % 8));
    }
}

//writes a string to the screen
void sh1106_WriteString(char* str, FontDef font, SH1106_COLOR color)
{
    //loop until the end of the string ('\0' character)
    while (*str)
    {
        //draw the character the pointer is currently pointing to
        sh1106_WriteChar(*str, font, color);

        //move the pointer to the next character in the string
        str++;
    }
}

//sets the cursor position
void sh1106_SetCursor(uint8_t x, uint8_t y) {
    currentX = x;
    currentY = y;
}

//SPI communication functions
void sh1106_WriteCommand(uint8_t command) {
    HAL_GPIO_WritePin(SH1106_CS_PORT, SH1106_CS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SH1106_DC_PORT, SH1106_DC_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&SH1106_SPI_PORT, &command, 1, 10);
    HAL_GPIO_WritePin(SH1106_CS_PORT, SH1106_CS_PIN, GPIO_PIN_SET);
}

void sh1106_WriteData(uint8_t* data, uint16_t size) {
    HAL_GPIO_WritePin(SH1106_CS_PORT, SH1106_CS_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SH1106_DC_PORT, SH1106_DC_PIN, GPIO_PIN_SET);
    HAL_SPI_Transmit(&SH1106_SPI_PORT, data, size, 100);
    HAL_GPIO_WritePin(SH1106_CS_PORT, SH1106_CS_PIN, GPIO_PIN_SET);
}

//writes a single character to the screen
char sh1106_WriteChar(char ch, FontDef font, SH1106_COLOR color) {
    // check if there is enough space on the screen to draw the character
    if (SH1106_WIDTH < (currentX + font.FontWidth) || SH1106_HEIGHT < (currentY + font.FontHeight)) {
        return 0; //not enough space
    }

    //iterates through each vertical column of the character
    for (int j = 0; j < font.FontWidth; j++) {
        //iterates through each pixel in the vertical column
        for (int i = 0; i < font.FontHeight; i++) {
            //check the bit for the current pixel and draw if it's set
            if ((font.data[((ch - 32) * font.FontWidth) + j] >> i) & 0x01) {
                sh1106_DrawPixel(currentX + j, currentY + i, color);
            }
        }
    }

    //moves the cursor to the right for the next character, adding a 1-pixel gap
    currentX += font.FontWidth + 1;

    return ch;
}
