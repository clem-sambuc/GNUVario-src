#if defined(ESP8266) || defined(ESP32)
#include <pgmspace.h>
#else
#include <avr/pgmspace.h>
#endif
// 24 x 24 gridicons_cart
const unsigned char gridicons_cart[] PROGMEM = { /* 0X01,0X01,0XB4,0X00,0X40,0X00, */
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
0xC7, 0xFF, 0xFF, 0x87, 0xFF, 0xFF, 0x9F, 0xE0, 
0x00, 0x1F, 0xE0, 0x00, 0x1F, 0xF0, 0x00, 0x1F, 
0xF0, 0x00, 0x1F, 0xF0, 0x00, 0x1F, 0xF0, 0x00, 
0x1F, 0xF8, 0x00, 0x1F, 0xF8, 0x00, 0x1F, 0xFF, 
0xFF, 0x9F, 0xFF, 0xFF, 0x9F, 0xFC, 0x00, 0x1F, 
0xF8, 0x00, 0x3F, 0xFF, 0xFF, 0xFF, 0xFC, 0xFF, 
0x3F, 0xF8, 0x7E, 0x1F, 0xF8, 0x7E, 0x1F, 0xFC, 
0xFF, 0x3F, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};
