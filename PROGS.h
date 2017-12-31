/**
 * MicroChess program by Peter Jennings
 */

#define _MICROCHESS_ENTRY 0x0000

const byte _MICROCHESS_0000[] PROGMEM = {
    /* 0x0000 */
    0xD8,0xA2,0xFF,0x9A, 0xA2,0xC8,0x86,0xB2, 0x20,0x1F,0x1F,0x20, 0x6A,0x1F,0xC5,0xF3, 
    0xF0,0xF6,0x85,0xF3, 0xC9,0x0C,0xD0,0x0F, 0xA2,0x1F,0xB5,0x70, 0x95,0x50,0xCA,0x10, 
    0xF9,0x86,0xDC,0xA9, 0xCC,0xD0,0x12,0xC9, 0x0E,0xD0,0x07,0x20, 0xB2,0x02,0xA9,0xEE, 
    0xD0,0x07,0xC9,0x14, 0xD0,0x0B,0x20,0xA2, 0x03,0x85,0xFB,0x85, 0xFA,0x85,0xF9,0xD0, 
    /* 0x0040 */
    0xBF,0xC9,0x0F,0xD0, 0x06,0x20,0x4B,0x03, 0x4C,0x9D,0x01,0x4C, 0x96, 0x01
};

const byte _MICROCHESS_0070[] PROGMEM = {
    /* 0x0070 */
    0x03,0x04,0x00,0x07, 0x02,0x05,0x01,0x06, 0x10,0x17,0x11,0x16, 0x12,0x15,0x14,0x13,
    /* 0x0080 */
    0x73,0x74,0x70,0x77, 0x72,0x75,0x71,0x76, 0x60,0x67,0x61,0x66, 0x62,0x65,0x64,0x63,
    0xF0,0xFF,0x01,0x10, 0x11,0x0F,0xEF,0xF1, 0xDF,0xE1,0xEE,0xF2, 0x12,0x0E,0x1F,0x21,
    0x0B,0x0A,0x06,0x06, 0x04,0x04,0x04,0x04, 0x02,0x02,0x02,0x02, 0x02,0x02,0x02,0x02,
    0x99,0x25,0x0B,0x25, 0x01,0x00,0x33,0x25, 0x07,0x36,0x34,0x0D, 0x34,0x34,0x0E,0x52,
    /* 0x00C0 */
    0x25,0x0D,0x45,0x35, 0x04,0x55,0x22,0x06, 0x43,0x33,0x0F,0xCC
};

const byte _MICROCHESS_0100[] PROGMEM = {
    /* 0x0100 */
    0xA6,0xB5,0x30,0x5C, 0xA5,0xB0,0xF0,0x08, 0xE0,0x08,0xD0,0x04, 0xC5,0xE6,0xF0,0x2E,
    0xF6,0xE3,0xC9,0x01, 0xD0,0x02,0xF6,0xE3, 0x50,0x1E,0xA0,0x0F, 0xA5,0xB1,0xD9,0x60,
    0x00,0xF0,0x03,0x88, 0x10,0xF8,0xB9,0xA0, 0x00,0xD5,0xE4,0x90, 0x04,0x94,0xE6,0x95,
    0xE4,0x18,0x08,0x75, 0xE5,0x95,0xE5,0x28, 0xE0,0x04,0xF0,0x03, 0x30,0x31,0x60,0xA5,
    /* 0x0140 */
    0xE8,0x85,0xDD,0xA9, 0x00,0x85,0xB5,0x20, 0x4B,0x03,0x20,0xB2, 0x02,0x20,0x00,0x02,
    0x20,0xB2,0x02,0xA9, 0x08,0x85,0xB5,0x20, 0x09,0x02,0x20,0x31, 0x03,0x4C,0x80,0x17,
    0xE0,0xF9,0xD0,0x0B, 0xA5,0x60,0xC5,0xB1, 0xD0,0x04,0xA9,0x00, 0x85,0xB4,0x60,0x50,
    0xFD,0xA0,0x07,0xA5, 0xB1,0xD9,0x60,0x00, 0xF0,0x05,0x88,0xF0, 0xF1,0x10,0xF6,0xB9,
    /* 0x0180 */
    0xA0,0x00,0xD5,0xE2, 0x90,0x02,0x95,0xE2, 0xC6,0xB5,0xA9,0xFB, 0xC5,0xB5,0xF0,0x03,
    0x20,0x25,0x03,0xE6, 0xB5,0x60,0xC9,0x08, 0xB0,0x12,0x20,0xEA, 0x03,0xA2,0x1F,0xB5,
    0x50,0xC5,0xFA,0xF0, 0x03,0xCA,0x10,0xF7, 0x86,0xFB,0x86,0xB0, 0x4C,0x00,0x00
};

const byte _MICROCHESS_0200[] PROGMEM = {
    /* 0x0200 */
    0xA2,0x10,0xA9,0x00, 0x95,0xDE,0xCA,0x10, 0xFB,0xA9,0x10,0x85, 0xB0,0xC6,0xB0,0x10,
    0x01,0x60,0x20,0x1E, 0x03,0xA4,0xB0,0xA2, 0x08,0x86,0xB6,0xC0, 0x08,0x10,0x41,0xC0,
    0x06,0x10,0x2E,0xC0, 0x04,0x10,0x1F,0xC0, 0x01,0xF0,0x09,0x10, 0x0E,0x20,0x8E,0x02,
    0xD0,0xFB,0xF0,0xD9, 0x20,0x9C,0x02,0xD0, 0xFB,0xF0,0xD2,0xA2, 0x04,0x86,0xB6,0x20,
    /* 0x0240 */
    0x9C,0x02,0xD0,0xFB, 0xF0,0xC7,0x20,0x9C, 0x02,0xA5,0xB6,0xC9, 0x04,0xD0,0xF7,0xF0,
    0xBC,0xA2,0x10,0x86, 0xB6,0x20,0x8E,0x02, 0xA5,0xB6,0xC9,0x08, 0xD0,0xF7,0xF0,0xAD,
    0xA2,0x06,0x86,0xB6, 0x20,0xCA,0x02,0x50, 0x05,0x30,0x03,0x20, 0x00,0x01,0x20,0x1E,
    0x03,0xC6,0xB6,0xA5, 0xB6,0xC9,0x05,0xF0, 0xEB,0x20,0xCA,0x02, 0x70,0x8F,0x30,0x8D,
    /* 0x0280 */
    0x20,0x00,0x01,0xA5, 0xB1,0x29,0xF0,0xC9, 0x20,0xF0,0xEE,0x4C, 0x0D,0x02,0x20,0xCA,
    0x02,0x30,0x03,0x20, 0x00,0x01,0x20,0x1E, 0x03,0xC6,0xB6,0x60, 0x20,0xCA,0x02,0x90,
    0x02,0x50,0xF9,0x30, 0x07,0x08,0x20,0x00, 0x01,0x28,0x50,0xF0, 0x20,0x1E,0x03,0xC6,
    0xB6,0x60,0xA2,0x0F, 0x38,0xB4,0x60,0xA9, 0x77,0xF5,0x50,0x95, 0x60,0x94,0x50,0x38,
    /* 0x02C0 */
    0xA9,0x77,0xF5,0x50, 0x95,0x50,0xCA,0x10, 0xEB,0x60,0xA5,0xB1, 0xA6,0xB6,0x18,0x75,
    0x8F,0x85,0xB1,0x29, 0x88,0xD0,0x42,0xA5, 0xB1,0xA2,0x20,0xCA, 0x30,0x0E,0xD5,0x50,
    0xD0,0xF9,0xE0,0x10, 0x30,0x33,0xA9,0x7F, 0x69,0x01,0x70,0x01, 0xB8,0xA5,0xB5,0x30,
    0x24,0xC9,0x08,0x10, 0x20,0x48,0x08,0xA9, 0xF9,0x85,0xB5,0x85, 0xB4,0x20,0x4B,0x03,
    
    /* 0x0300 */
    0x20,0xB2,0x02,0x20, 0x09,0x02,0x20,0x2E, 0x03,0x28,0x68,0x85, 0xB5,0xA5,0xB4,0x30,
    0x04,0x38,0xA9,0xFF, 0x60,0x18,0xA9,0x00, 0x60,0xA9,0xFF,0x18, 0xB8,0x60,0xA6,0xB0,
    0xB5,0x50,0x85,0xB1, 0x60,0x20,0x4B,0x03, 0x20,0xB2,0x02,0x20, 0x09,0x02,0x20,0xB2,
    0x02,0xBA,0x86,0xB3, 0xA6,0xB2,0x9A,0x68, 0x85,0xB6,0x68,0x85, 0xB0,0xAA,0x68,0x95,
    /* 0x0340 */
    0x50,0x68,0xAA,0x68, 0x85,0xB1,0x95,0x50, 0x4C,0x70,0x03,0xBA, 0x86,0xB3,0xA6,0xB2,
    0x9A,0xA5,0xB1,0x48, 0xA8,0xA2,0x1F,0xD5, 0x50,0xF0,0x03,0xCA, 0x10,0xF9,0xA9,0xCC,
    0x95,0x50,0x8A,0x48, 0xA6,0xB0,0xB5,0x50, 0x94,0x50,0x48,0x8A, 0x48,0xA5,0xB6,0x48,
    0xBA,0x86,0xB2,0xA6, 0xB3,0x9A,0x60,0xA6, 0xE4,0xE4,0xA0,0xD0, 0x04,0xA9,0x00,0xF0,
    /* 0x0380 */
    0x0A,0xA6,0xE3,0xD0, 0x06,0xA6,0xEE,0xD0, 0x02,0xA9,0xFF,0xA2, 0x04,0x86,0xB5,0xC5,
    0xFA,0x90,0x0C,0xF0, 0x0A,0x85,0xFA,0xA5, 0xB0,0x85,0xFB,0xA5, 0xB1,0x85,0xF9,0x4C,
    0x1F,0x1F,0xA6,0xDC, 0x10,0x17,0xA5,0xF9, 0xD5,0xDC,0xD0,0x0F, 0xCA,0xB5,0xDC,0x85,
    0xFB,0xCA,0xB5,0xDC, 0x85,0xF9,0xCA,0x86, 0xDC,0xD0,0x1A,0x85, 0xDC,0xA2,0x0C,0x86,
    /* 0x03C0 */
    0xB5,0x86,0xFA,0xA2, 0x14,0x20,0x02,0x02, 0xA2,0x04,0x86,0xB5, 0x20,0x00,0x02,0xA6,
    0xFA,0xE0,0x0F,0x90, 0x12,0xA6,0xFB,0xB5, 0x50,0x85,0xFA,0x86, 0xB0,0xA5,0xF9,0x85,
    0xB1,0x20,0x4B,0x03, 0x4C,0x00,0x00,0xA9, 0xFF,0x60,0xA2,0x04, 0x06,0xF9,0x26,0xFA,
    0xCA,0xD0,0xF9,0x05, 0xF9,0x85,0xF9,0x85, 0xB1,0x60
};

const byte _MICROCHESS_1780[] PROGMEM = {
    /* 0x1780 */
    0x18,0xA9,0x80,0x65, 0xEB,0x65,0xEC,0x65, 0xED,0x65,0xE1,0x65, 0xDF,0x38,0xE5,0xF0,
    0xE5,0xF1,0xE5,0xE2, 0xE5,0xE0,0xE5,0xDE, 0xE5,0xEF,0xE5,0xE3, 0xB0,0x02,0xA9,0x00,
    0x4A,0x18,0x69,0x40, 0x65,0xEC,0x65,0xED, 0x38,0xE5,0xE4,0x4A, 0x18,0x69,0x90,0x65,
    0xDD,0x65,0xDD,0x65, 0xDD,0x65,0xDD,0x65, 0xE1,0x38,0xE5,0xE4, 0xE5,0xE4,0xE5,0xE5,
    /* 0x17C0 */
    0xE5,0xE5,0xE5,0xE0, 0xA6,0xB1,0xE0,0x33, 0xF0,0x16,0xE0,0x34, 0xF0,0x12,0xE0,0x22,
    0xF0,0x0E,0xE0,0x25, 0xF0,0x0A,0xA6,0xB0, 0xF0,0x09,0xB4,0x50, 0xC0,0x10,0x10,0x03,
    0x18,0x69,0x02,0x4C, 0x77,0x03
};

/**
 * Blackjack program from "The First Book of KIM"
 */

#define _BLACKJACK_ENTRY 0x0200

const byte _BLACKJACK[] PROGMEM = { 
    /* 0x0200 */
    0xA2,0x33,0x8A,0x95, 0x40,0xCA,0x10,0xFA, 0xA2,0x02,0xBD,0xBB, 0x03,0x95,0x75,0xCA,
    0x10,0xF8,0xAD,0x04, 0x17,0x85,0x80,0xD8, 0xA6,0x76,0xE0,0x09, 0xB0,0x34,0xA0,0xD8,
    0x20,0x57,0x03,0xA0, 0x33,0x84,0x76,0x20, 0x30,0x03,0x38,0xA5, 0x81,0x65,0x82,0x65,
    0x85,0x85,0x80,0xA2, 0x04,0xB5,0x80,0x95, 0x81,0xCA,0x10,0xF9, 0x29,0x3F,0xC9,0x34,
    /* 0x0240 */
    0xB0,0xE5,0xAA,0xB9, 0x40,0x00,0x48,0xB5, 0x40,0x99,0x40,0x00, 0x68,0x95,0x40,0x88,
    0x10,0xD5,0xA0,0xDE, 0x20,0x57,0x03,0xA5, 0x77,0x20,0xA6,0x03, 0x20,0x30,0x03,0xC9,
    0x0A,0xB0,0xF9,0xAA, 0x86,0x79,0xCA,0x30, 0xF3,0xE4,0x77,0xB0, 0xEF,0xA2,0x0B,0xA9,
    0x00,0x95,0x90,0xCA, 0x10,0xFB,0x20,0x78, 0x03,0x20,0x8F,0x03, 0x20,0x78,0x03,0x20,
    /* 0x0280 */
    0x64,0x03,0x86,0x7A, 0x20,0x28,0x03,0x20, 0x30,0x03,0xAA,0xCA, 0x30,0x11,0xE4,0x96,
    0xD0,0xF5,0x20,0x78, 0x03,0xC9,0x22,0xB0, 0x40,0xE0,0x05,0xF0, 0x53,0xD0,0xE8,0xA5,
    0x95,0x48,0xA2,0x00, 0x20,0x0F,0x03,0xA2, 0x04,0xA9,0x00,0x95, 0x90,0xCA,0x10,0xFB,
    0x68,0x85,0x95,0xA6, 0x7A,0x20,0x6D,0x03, 0x20,0x92,0x03,0x20, 0x28,0x03,0xA5,0x9A,
    /* 0x02C0 */
    0xC9,0x22,0xB0,0x29, 0x65,0x9B,0xA6,0x91, 0xD0,0x18,0xC9,0x22, 0x90,0x02,0xA5,0x9A,
    0xC9,0x17,0xB0,0x2C, 0x20,0x8F,0x03,0xD0, 0xE2,0x20,0x28,0x03, 0x20,0x55,0x03,0x20,
    0x28,0x03,0xA5,0x77, 0xF8,0x38,0xE5,0x79, 0x85,0x77,0x4C,0x17, 0x02,0x20,0x55,0x03,
    0x20,0x28,0x03,0xA5, 0x77,0xF8,0x18,0x65, 0x79,0xA0,0x99,0x90, 0x01,0x98,0xD0,0xE8,
    
    /* 0x0300 */
    0xA2,0x03,0x20,0x0F, 0x03,0xA5,0x9A,0xC5, 0x97,0xF0,0xDF,0xB0, 0xD5,0x90,0xE4,0xB5,
    0x97,0xF8,0x18,0x75, 0x98,0xC9,0x22,0xB0, 0x02,0x95,0x97,0xD8, 0xB5,0x97,0x48,0xA0,
    0xE2,0x20,0x57,0x03, 0x68,0x20,0xA6,0x03, 0xA0,0x80,0x20,0x30, 0x03,0x88,0xD0,0xFA,
    0x84,0x7F,0xA0,0x13, 0xA2,0x05,0xA9,0x7F, 0x8D,0x41,0x17,0xB5, 0x90,0x8D,0x40,0x17,
    /* 0x0340 */
    /* Change unitialized delay loop to EMT 16                                      */
    /*               vv    vv                                                       */
    0x8C,0x42,0x17,0x0F, 0x10,0xD0,0xFC,0x88, 0x88,0xCA,0x10,0xEF, 0x20,0x40,0x1F,0x20,
    0x6A,0x1F,0xA4,0x7F, 0x60,0xA0,0xE6,0x84, 0x74,0xA0,0x05,0xB1, 0x74,0x99,0x90,0x00,
    0x88,0x10,0xF8,0x60, 0xA6,0x76,0xC6,0x76, 0xB5,0x40,0x4A,0x4A, 0xAA,0x18,0xD0,0x01,
    0x38,0xBD,0xBE,0x03, 0xBC,0xCB,0x03,0x60, 0x20,0x64,0x03,0xE6, 0x96,0xA6,0x96,0x94,
    /* 0x0380 */
    0x8F,0xA0,0x10,0x90, 0x02,0x84,0x98,0x18, 0xF8,0x65,0x97,0x85, 0x97,0xD8,0x60,0x20,
    0x64,0x03,0xC6,0x99, 0xA6,0x99,0x94,0x96, 0xA0,0x10,0x90,0x02, 0x84,0x9B,0x18,0xF8,
    0x65,0x9A,0x85,0x9A, 0xD8,0x60,0x48,0x4A, 0x4A,0x4A,0x4A,0xA8, 0xB9,0xE7,0x1F,0x85,
    0x94,0x68,0x29,0x0F, 0xA8,0xB9,0xE7,0x1F, 0x85,0x95,0x60,0x03, 0x00,0x20,0x01,0x02,
    /* 0x03C0 */
    0x03,0x04,0x05,0x06, 0x07,0x08,0x09,0x10, 0x10,0x10,0x10,0xF7, 0xDB,0xCF,0xE6,0xED,
    0xFD,0x87,0xFF,0xEF, 0xF1,0xF1,0xF1,0xF1, 0xED,0xF6,0xBE,0xF1, 0xF1,0xB8,0xFC,0xF9,
    0xF8,0xD3,0xF8,0xDC, 0xF8,0xC0,0xFC,0xBE, 0xED,0x87,0xF9,0xDE, 0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00                                          
 };

/**
 * Wumpus program from "The First Book of KIM"
 */

#define _WUMPUS_ENTRY 0x0305

const byte _WUMPUS_0000[] PROGMEM = {
    /* 0x0000 */
    0x80,0xEE,0xDC,0xBE, 0x80,0xF7,0xD0,0xF9, 0x80,0x84,0xD4,0x80, 0xEF,0x80,0xC0,0x80,
    0xF8,0xBE,0xD4,0xD4, 0xF9,0xB8,0xED,0x80, 0xB8,0xF9,0xF7,0xDE, 0x80,0xF8,0xDC,0x80,
    0xFD,0xFF,0xF7,0xB9, 0x80,0x00,0x80,0xDC, 0xDC,0xF3,0xED,0x80, 0xC0,0x80,0xFC,0xBE,
    0xB7,0xF3,0xF9,0xDE, 0x80,0xF7,0x80,0x9C, 0xBE,0xB7,0xF3,0xBE, 0xED,0x80,0x80,0x00
};

const byte _WUMPUS_0050[] PROGMEM = {
    /* 0x0050 */
    0x02,0x02,0x00,0x01, 0x01,0x00,0x03,0x04, 0x00,0x06,0x07,0x00, 0x09,0x0A,0x01,0x04,
    0x05,0x03,0x01,0x02, 0x03,0x02,0x05,0x06, 0x05,0x08,0x09,0x08, 0x0B,0x0C,0x0B,0x07,
    0x08,0x04,0x03,0x04, 0x07,0x06,0x07,0x0A, 0x09,0x0A,0x0F,0x0C, 0x0D,0x0E,0x0C,0x0A,
    /* 0x0080 */
    0x0B,0x0E,0x05,0x06, 0x0F,0x08,0x09,0x0F, 0x0B,0x0C,0x0D,0x0E, 0x0E,0x0F,0x0D,0x0D,
    0x80,0xB7,0x84,0xED, 0xED,0xF9,0xDE,0x80, 0xC0,0x80,0xDC,0xD4, 0xB8,0xEE,0x80,0xDB,
    0x80,0xB9,0xF7,0xD4, 0xED,0x80,0xB8,0xF9, 0xF1,0xF8,0x80,0x00, 0x80,0xEE,0xDC,0xBE,
    0x80,0xB8,0xDC,0xED, 0xF9,0x80,0x00,0x80, 0xD0,0xDC,0xDC,0xB7, 0xD3,0x80,0x00,0x03
};

const byte _WUMPUS_0100[] PROGMEM = {
    /* 0x0100 */
    0x80,0x9C,0xBE,0xB7, 0xF3,0xBE,0xED,0x80, 0xB9,0xB8,0xDC,0xED, 0xF9,0x00,0x80,0xF3,
    0x84,0xF8,0x80,0xB9, 0xB8,0xDC,0xED,0xF9, 0x00,0x80,0xFC,0xF7, 0xF8,0xED,0x80,0xB9,
    0xB8,0xDC,0xED,0xF9, 0x80,0x00,0x80,0xF6, 0xF7,0x80,0xF6,0xF7, 0x80,0x9C,0xBE,0xB7,
    0xF3,0xBE,0xED,0x80, 0xBD,0xDC,0xF8,0x80, 0xEE,0xDC,0xBE,0x80, 0x00,0x80,0xED,0xBE,
    /* 0x0140 */
    0xF3,0xF9,0xD0,0xFC, 0xF7,0xF8,0x80,0xED, 0xD4,0xF7,0xF8,0xB9, 0xF6,0x80,0x00,0x80,
    0xEE,0xEE,0x84,0x84, 0xF9,0xF9,0xF9,0x80, 0xF1,0xF9,0xB8,0xB8, 0x80,0x84,0xD4,0x80,
    0xF3,0x84,0xF8,0x80, 0x00,0x80,0xBD,0xF7, 0xED,0x80,0x84,0xD4, 0x80,0xD0,0xDC,0xDC,
    0xB7,0x80,0x00,0x80, 0xDC,0xBE,0xF8,0x80, 0xDC,0xF1,0x80,0xBD, 0xF7,0xED,0x80,0x00,
    /* 0x0180 */
    0x80,0x80,0x80,0x80, 0x80,0xBD,0xD0,0xF9, 0xF7,0xF8,0xC0,0x80, 0xEE,0xDC,0xBE,0x80,
    0xBD,0xF9,0xF8,0x80, 0xF7,0x80,0xF6,0xBE, 0xBD,0x80,0xF1,0xD0, 0xDC,0xB7,0x80,0x9C,
    0xBE,0xB7,0xF3,0xBE, 0xED,0x80,0x00 
};

const byte _WUMPUS_0200[] PROGMEM = {
    /* 0x0200 */
    0x84,0xDE,0x85,0xDD, 0xA9,0x07,0x85,0xDF, 0xA0,0x05,0xA2,0x05, 0xB1,0xDD,0xC9,0x00,
    0xD0,0x01,0x60,0x95, 0xE8,0x88,0xCA,0x10, 0xF3,0xD8,0x18,0x98, 0x65,0xDF,0x85,0xDC,
    /*                           Lower delay loop value    Lower timer value        */
    /*                                               vv                   vv        */
    0x20,0x28,0x02,0xA4, 0xDC,0x4C,0x0A,0x02, 0xA2,0x02,0x86,0xDB, 0xA9,0x04,0x8D,0x07,
    0x17,0x20,0x3E,0x02, 0x2C,0x07,0x17,0x10, 0xF8,0xC6,0xDB,0xD0, 0xEF,0x60,0xA9,0x7F,
    /* 0x0240 */
    0x8D,0x41,0x17,0xA0, 0x00,0xA2,0x09,0xB9, 0xE8,0x00,0x84,0xFC, 0x20,0x4E,0x1F,0xC8,
    0xC0,0x06,0x90,0xF3, 0x20,0x3D,0x1F,0x60, 0x20,0x8C,0x1E,0x20, 0x3E,0x02,0xD0,0xF8,
    0x20,0x3E,0x02,0xF0, 0xFB,0x20,0x3E,0x02, 0xF0,0xF6,0x20,0x6A, 0x1F,0xC9,0x15,0x10,
    0xE7,0x60,0x8A,0x48, 0xD8,0x38,0xA5,0x41, 0x65,0x44,0x65,0x45, 0x85,0x40,0xA2,0x04,
    /* 0x0280 */
    0xB5,0x40,0x95,0x41, 0xCA,0x10,0xF9,0x85, 0xC0,0x68,0xAA,0xA5, 0xC0,0x60,0x60,0xA2,
    0x04,0xD5,0xCB,0xF0, 0x03,0xCA,0x10,0xF9, 0x60,0x20,0x72,0x02, 0x29,0x0F,0xC9,0x04,
    0x30,0x0D,0x20,0xB2, 0x02,0xAD,0x06,0x17, 0x29,0x03,0xAA,0xB5, 0xC6,0x85,0xCB,0xA5,
    0xCB,0x60,0xA6,0xCA, 0xB5,0x50,0x85,0xC6, 0xB5,0x60,0x85,0xC7, 0xB5,0x70,0x85,0xC8,
    /* 0x02C0 */
    0xB5,0x80,0x85,0xC9, 0x60,0xA2,0x03,0xD5, 0xC6,0xF0,0x03,0xCA, 0x10,0xF9,0x60,0xA0,
    0x01,0x20,0x00,0x02, 0xA0,0x00,0xA9,0xAC, 0x20,0x00,0x02,0x4C, 0xD4,0x02,0xA4,0xE0,
    0xB9,0xE7,0x1F,0x85, 0x9F,0xA0,0x00,0xA9, 0x90,0x20,0x00,0x02, 0x4C,0x2C,0x03,0xF6,
    0xBE,0xBD,0x80,0xF1, 0xD0,0xDC,0xB7,0x80, 0x9C,0xBE,0xB7,0xF3, 0xBE,0xED,0x80,0x00,

    /* 0x0300 */
    0xEA,0xEA,0xEA,0xEA, 0xEA,0xA9,0xFF,0xA2, 0x0E,0x95,0xC1,0xCA, 0x10,0xFB,0xA9,0x03,
    0x85,0xE0,0xA0,0x05, 0x10,0x02,0xA0,0x00, 0xA2,0x05,0x20,0x72, 0x02,0x29,0x0F,0xD5,
    0xCA,0xF0,0xF5,0xCA, 0x10,0xF9,0x99,0xCA, 0x00,0x88,0x10,0xEC, 0x20,0xB2,0x02,0xA0,
    0x03,0x84,0xE1,0xB9, 0xC6,0x00,0x20,0x8F, 0x02,0x8A,0x30,0x17, 0xE0,0x03,0x30,0x04,
    /* 0x0340 */
    0xA9,0x19,0x10,0x0A, 0xE0,0x01,0x30,0x04, 0xA9,0x0E,0x10,0x02, 0xA9,0x00,0xA0,0x01,
    0x20,0x00,0x02,0xC6, 0xE1,0xA4,0xE1,0x10, 0xDA,0xA4,0xCA,0xB9, 0xE7,0x1F,0x85,0x0C,
    0xA2,0x03,0xB4,0xC6, 0xB9,0xE7,0x1F,0x95, 0x20,0xCA,0x10,0xF6, 0xA0,0x00,0x98,0x20,
    0x00,0x02,0x20,0x58, 0x02,0xC9,0x14,0xF0, 0x48,0x20,0xC5,0x02, 0x85,0xCA,0x8A,0x30,
    /* 0x0380 */
    0xEB,0xA5,0xCA,0xA2, 0x04,0xD5,0xC1,0xF0, 0x33,0xCA,0x10,0xF9, 0x20,0x8F,0x02,0x8A,
    0x30,0x9A,0xE0,0x03, 0x10,0x17,0xE0,0x01, 0x10,0x1D,0xA0,0x00, 0xA9,0x26,0x20,0x00,
    0x02,0x20,0x99,0x02, 0xC5,0xCA,0xD0,0x84, 0xA9,0x26,0x4C,0xCF, 0x02,0xA0,0x01,0xA9,
    0x3D,0x20,0x00,0x02, 0x4C,0x16,0x03,0xA9, 0x4F,0x4C,0xCF,0x02, 0xA9,0x65,0x4C,0xCF,
    /* 0x03C0 */
    0x02,0xA0,0x00,0xA9, 0xB7,0x20,0x00,0x02, 0x20,0x58,0x02,0x20, 0xC5,0x02,0x85,0xD1,
    0x8A,0x30,0xEE,0xA5, 0xD1,0xA6,0xE0,0x95, 0xC0,0xC5,0xCB,0xF0, 0x15,0xC6,0xE0,0xF0,
    0x1A,0xA6,0xCB,0x20, 0xB4,0x02,0x20,0xA5, 0x02,0xC5,0xCA,0xF0, 0xBB,0x4C,0xDE,0x02,
    0xEA,0xEA,0xA0,0x01, 0xA9,0x80,0x20,0x00, 0x02,0xF0,0xF7,0xA9, 0x73,0x20,0xCF,0x02
}; 

/**
 * Asteroid program from "The First Book of KIM"
 */

#define _ASTEROID_ENTRY 0x0200

const byte _ASTEROID[] PROGMEM = {
    /* 0x0200 */
    0xA9,0x00,0x85,0xF9, 0x85,0xFA,0x85,0xFB, 0xA2,0x06,0xBD,0xCE, 0x02,0x95,0xE2,0xCA,
    0x10,0xF8,0xA5,0xE8, 0x49,0xFF,0x85,0xE8, 0xA2,0x05,0x20,0x48, 0x02,0x20,0x97,0x02,
    0xCA,0xD0,0xF7,0x20, 0x40,0x1F,0x20,0x6A, 0x1F,0xC9,0x15,0x10, 0xE5,0xC9,0x00,0xF0,
    0x06,0xC9,0x03,0xF0, 0x0A,0xD0,0xDB,0x06, 0xE7,0xA9,0x40,0xC5, 0xE7,0xD0,0xD3,0x46,
    /* 0x0240 */
    0xE7,0xD0,0xCF,0x38, 0x26,0xE7,0xD0,0xCA, 0xA9,0x7F,0x8D,0x41, 0x17,0xA9,0x09,0x8D,
    0x42,0x17,0xA9,0x20, 0x85,0xE0,0xA0,0x02, 0xA9,0x00,0x85,0xE1, 0xB1,0xE2,0x25,0xE0,
    0xF0,0x07,0xA5,0xE1, 0x19,0xE4,0x00,0x85, 0xE1,0x88,0x10,0xF0, 0xA5,0xE1,0xC4,0xE8,
    0xD0,0x08,0xA4,0xE0, 0xC4,0xE7,0xD0,0x02, 0x09,0x08,0x8D,0x40, 0x17,0xA9,0x30,0x8D,
    /* 0x0280 */
    0x06,0x17,0xAD,0x07, 0x17,0xF0,0xFB,0xA9, 0x00,0x8D,0x40,0x17, 0xEE,0x42,0x17,0xEE,
    0x42,0x17,0x46,0xE0, 0xD0,0xC0,0x60,0xC6, 0xE9,0xD0,0x1A,0xA9, 0x30,0x85,0xE9,0x8A,
    0x48,0xA2,0xFD,0xF8, 0x38,0xB5,0xFC,0x69, 0x00,0x95,0xFC,0xE8, 0xD0,0xF7,0xD8,0x68,
    0xAA,0xE6,0xE2,0xA5, 0xE2,0xC9,0x30,0xF0, 0x09,0xA0,0x00,0xA5, 0xE7,0x31,0xE2,0xD0,
    /* 0x02C0 */
    0x07,0x60,0xA9,0x00, 0x85,0xE2,0xF0,0xF1, 0x20,0x1F,0x1F,0x4C, 0xC8,0x02,0xD5,0x02,
    0x08,0x40,0x01,0x04, 0xFF,0x00,0x00,0x00, 0x04,0x00,0x08,0x00, 0x06,0x12,0x00,0x11,
    0x00,0x05,0x00,0x2C, 0x00,0x16,0x00,0x29, 0x00,0x16,0x00,0x2B, 0x00,0x26,0x00,0x19,
    0x00,0x17,0x00,0x38, 0x00,0x2E,0x00,0x09, 0x00,0x1B,0x00,0x24, 0x00,0x15,0x00,0x39,

    /* 0x0300 */
    0x00,0x0D,0x00,0x21, 0x00,0x10,0x00,0x00
}; 

/**
 * Clock program from "The First Book of KIM"
 */

#define _CLOCK_ENTRY 0x0300

const byte _CLOCK[] PROGMEM = {
    /* 0x0300 */
    0x20,0x6A,0x1F,0xC9, 0x01,0xD0,0x0D,0x20, 0x1F,0x1F,0x20,0x6A, 0x1F,0xC9,0x01,0xD0,
    0x03,0x4C,0x05,0x1C, 0x60,0x00,0x00,0x00, 0x00,0x00,0x00,0x00, 0x00,0x00,0x00,0x00,
    0xA5,0x82,0xD0,0x29, 0xA5,0x81,0x38,0xE5, 0x83,0x10,0x24,0xA5, 0x80,0xD0,0x06,0xA9,
    0x1E,0x85,0x70,0xD0, 0x0A,0xA9,0x01,0xC5, 0x80,0xD0,0x14,0xA9, 0x28,0x85,0x70,0xA9,
    /* 0x0340 */
    0x01,0x8D,0x03,0x17, 0xEE,0x02,0x17,0xA5, 0x70,0xAA,0xCA,0x10, 0xFD,0x30,0xDC,0x60,
    0x48,0x8A,0x48,0x98, 0x48,0xA9,0x83,0x8D, 0x04,0x17,0x20,0xC0, 0x17,0x10,0xFB,0xE6,
    0x80,0xA9,0x04,0xC5, 0x80,0xD0,0x38,0xA9, 0x00,0x85,0x80,0x18, 0xF8,0xA5,0x81,0x69,
    0x01,0x85,0x81,0xC9, 0x60,0xD0,0x28,0xA9, 0x00,0x85,0x81,0xA5, 0x82,0x18,0x69,0x01,
    /* 0x0380 */
    0x85,0x82,0xC9,0x60, 0xD0,0x19,0xA9,0x00, 0x85,0x82,0xA5,0x83, 0x18,0x69,0x01,0x85,
    0x83,0xC9,0x12,0xD0, 0x02,0xE6,0x84,0xC9, 0x13,0xB0,0x04,0xA9, 0x01,0x85,0x83,0xD8,
    0xA9,0xF4,0x8D,0x0F, 0x17,0x68,0xA8,0x68, 0xAA,0x68,0x40,0x00, 0x00,0x00,0x00,0x00,
    0xA9,0x00,0x85,0x80, 0xA9,0xF4,0x8D,0x0F, 0x17,0xA5,0x81,0x85, 0xF9,0xA5,0x82,0x85,
    /* 0x03C0 */
    0xFA,0xA5,0x83,0x85, 0xFB,0x20,0x1F,0x1F, 0x20,0x00,0x03,0x20, 0x20,0x03,0xEA,0xEA,
    0xEA,0xEA,0xEA,0xEA, 0xEA,0xEA,0xEA,0xEA, 0xEA,0xEA,0xEA,0xEA, 0xEA,0xEA,0xEA,0xEA,
    0xEA,0xEA,0xEA,0xEA, 0xEA,0xEA,0xEA,0xEA, 0xEA,0xEA,0xEA,0xEA, 0x4C,0xC9,0x03
};

