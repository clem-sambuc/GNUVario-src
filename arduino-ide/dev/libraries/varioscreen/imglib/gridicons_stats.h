#if defined(ESP8266) || defined(ESP32)
#include <pgmspace.h>
#else
#include <avr/pgmspace.h>
#endif
// 24 x 24 gridicons_stats
const unsigned char gridicons_stats[] PROGMEM = { /* 0X01,0X01,0XB4,0X00,0X40,0X00, */
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 
0xFF, 0xF0, 0x00, 0x0F, 0xE0, 0x00, 0x07, 0xE7, 
0xFF, 0xE7, 0xE7, 0xFF, 0xE7, 0xE7, 0xE7, 0xE7, 
0xE7, 0xE7, 0xE7, 0xE7, 0xE7, 0xE7, 0xE6, 0x67, 
0xE7, 0xE6, 0x67, 0xE7, 0xE6, 0x66, 0x67, 0xE6, 
0x66, 0x67, 0xE6, 0x66, 0x67, 0xE6, 0x66, 0x67, 
0xE6, 0x66, 0x67, 0xE7, 0xFF, 0xE7, 0xE7, 0xFF, 
0xE7, 0xE0, 0x00, 0x07, 0xF0, 0x00, 0x0F, 0xFF, 
0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF
};
