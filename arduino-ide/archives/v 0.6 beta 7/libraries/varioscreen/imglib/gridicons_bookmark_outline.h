#if defined(ESP8266) || defined(ESP32)
#include <pgmspace.h>
#else
#include <avr/pgmspace.h>
#endif
// 24 x 24 gridicons_bookmark_outline
const unsigned char gridicons_bookmark_outline[] PROGMEM = { /* 0X01,0X01,0XB4,0X00,0X40,0X00, */
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
0xFF, 0xFC, 0x00, 0x3F, 0xF8, 0x00, 0x1F, 0xF9, 
0xFF, 0x9F, 0xF9, 0xFF, 0x9F, 0xF9, 0xFF, 0x9F, 
0xF9, 0xFF, 0x9F, 0xF9, 0xFF, 0x9F, 0xF9, 0xFF, 
0x9F, 0xF9, 0xFF, 0x9F, 0xF9, 0xFF, 0x9F, 0xF9, 
0xFF, 0x9F, 0xF9, 0xFF, 0x9F, 0xF9, 0xE7, 0x9F, 
0xF9, 0x81, 0x9F, 0xF8, 0x18, 0x1F, 0xF8, 0x7E, 
0x1F, 0xF8, 0xFF, 0x1F, 0xFB, 0xFF, 0xDF, 0xFF, 
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};
