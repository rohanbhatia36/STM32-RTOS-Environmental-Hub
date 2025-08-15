#ifndef FONTS_H
#define FONTS_H

#include <stdint.h>

typedef struct {
    const uint8_t FontWidth;
    uint8_t FontHeight;
    const uint8_t *data;
} FontDef;

//declaration for the 7x10 font
extern FontDef Font_7x10;

#endif //FONTS_H
