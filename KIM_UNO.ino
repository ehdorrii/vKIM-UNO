/**
 *
 * KIM_UNO.ino - KIM-1 emulation
 *
 * This is a rehash of my PalmOS KIM-1 emulator (which I called
 * vKIM) to work with the Arduino-Mini in Oscar Vermeulen's 
 * KIM-UNO. 
 *
 * --------------------------------------------------------------
 *
 * Copyright (c) 2017 Eugene Dorr
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <avr/pgmspace.h>
#include <EEPROM.h>

///////
// Trace options for debugging
///////

boolean  TRACE_REGISTERS;
boolean  TRACE_INSTRUCTIONS;
boolean  TRACE_INSTRUCTION_TIMING;
boolean  TRACE_MEMORY_ACCESS;
boolean  TRACE_IO;
boolean  TRACE_HW_LINES;
boolean  TRACE_ROM;
uint16_t TRACE_DELAY;
unsigned long BEGIN_TIME;
unsigned long EXCLUDE_TIME;
unsigned long EXECUTION_TIME;

///////
// Symbolic addresses for values stored in EEPROM
///////
 
#define eeaLED   0
#define eeaTTY   4

///////
// Meta-registers
///////

/**
 * These are not part of the 6502 architecture, but are used by the C
 * routines which implement the 6502. Anyone who has some familiarity
 * with microcode will find these registers familiar.
 */

uint8_t  regMDR;        // memory data register
uint16_t regMAR;        // memory address register
uint8_t  regOP;         // current opcode
uint8_t  regALUin;      // represents a single ALU input
uint16_t regALUout;     // represents the ALU output
uint8_t  regHC;         // represents ALU half-carry/half-borrow

// This is a helper variable for accessing tables stored in AVR PROGMEM:
uint8_t  opIx;         // index into tables organized by opcode

#define bit0 0b00000001
#define bit1 0b00000010
#define bit2 0b00000100
#define bit3 0b00001000
#define bit4 0b00010000
#define bit5 0b00100000
#define bit6 0b01000000
#define bit7 0b10000000

/**
 * The implementation of cycle counting is not perfect, but it's probably
 * good enough for most purposes.
 */

uint32_t hwCycles;       // cycles since mpuInit()
uint32_t avrMicros;      // AVR clock -- used by EMT
boolean  PBC = false;    // Page-boundary-crossed flag: additional clock cycle used
uint8_t  carry;
uint8_t  borrow;

///////
// 6502 Programmer registers
///////

uint8_t  regA;      // accumulator
uint8_t  regX;      // index register X
uint8_t  regY;      // index register Y
uint8_t  regSP;     // stack pointer (implicitly located in page 1)
uint16_t regPC;     // program counter
uint8_t  regPS;     // status register

#define psN 0x80    // negative
#define psV 0x40    // overflow
#define psB 0x10    // break
#define psD 0x08    // decimal mode
#define psI 0x04    // interrupt mask
#define psZ 0x02    // zero
#define psC 0x01    // carry

/**
 * Opcode tables used by the 6502 emulation routines
 */
 
#include "OP_TABLES.h"

///////
// Hardware lines 
///////

boolean hwRDY;      // ready -- ok to continue executing
boolean hwRST;      // reset
boolean hwNMI;      // non-maskable interrupt
boolean hwIRQ;      // interrupt

///////
// BCD Decoder
///////

uint8_t U24_74145[] = {9, 10, 11, 255, 12, 13, A0, A1, A2, A3, 255, 255, 255, 255, 255, 255};  
const uint8_t ledGap[]   PROGMEM = {12, 13, A0, A1};
const uint8_t ledNoGap[] PROGMEM = {13, A0, A1, A4};

///////
// KIM-1 SST switch state, and TTY jumper state
///////

boolean kimSST = false;        // Toggled by the 'SST' key
boolean kimTTY = false;        // Set by holding down '0' key during power-on

///////
// Memory 
///////

/**
 * NOTE that on a basic KIM-1 the high 3 address lines are not decoded! Thus the
 * total address space is only 8K. (Which is why the monitor is assembled for the
 * range 0x1800..0x1FFF.)
 *
 * The memory map for an unmodified KIM-1 is:
 *
 *    pages    locations   implemented in    usage
 *    ------   ---------   ---------------   ---------------------------------
 *    0-3      0000-03FF   8 x 6102 RAM      RAM
 *    4-19     0400-13FF   not implemented   decoded - available for expansion
 *    20-22    1400-16FF   not implemented   no decoding hardware present
 *    23       1700-173F   6530-003          Application I/O (A5 & A4 not decoded)
 *    23       1740-177F   6530-002          System I/O (A5 & A4 not decoded)
 *    23       1780-17BF   6530-003          RAM
 *    23       17C0-17FF   6530-002          RAM
 *    24-27    1800-1BFF   6530-003          ROM
 *    28-31    1C00-1FFF   6530-002          ROM
 *    32-128   2000-FFFF   not implemented   not decoded - unavailable for use
 *
 * The Arduino Pro Mini used in the KIM-UNO doesn't have enough SRAM to emulate
 * more than 1K of RAM, so this program sticks with the standard KIM-1 RAM size.
 */

#define pageZeroMask 0x00ff;
#define RAMsize 1024
uint8_t memRAM[RAMsize];

/**
 * The following block of RAM is peculiar to the KIM-1:
 *  - addresses 0x1780..0x17BF are in 6530-003
 *  - addresses 0x17C0..0x17FF are in 6530-002
 */

#define auxRAMsize 128
#define auxRAMstart 0x1780
uint8_t auxRAM[auxRAMsize];

/**
 * In the KIM-1 the following address ranges access 6530 I/O & timer functions:
 *
 *  - 0x1700..0x173F is in 6530-003 (available for application use). The Arduino
 *    Pro Mini used in the KIM-UNO doesn't provide enough ports to map the KIM-1's
 *    Port B, so it isn't emulated. But the timer *is* emulated.
 *
 *  - 0x1740..0x177F is in 6530-002 (reserved for system use). The I/O in this
 *    range is used to interface the keypad and LED display. Timer emulated.
 *
 *  ==Notes regarding 6530 I/O and Timer area==
 *
 *  A5 & A4 are not decoded. As a result, it appears as if the 
 *  following is repeated 4 times in the memory map of a RIOT.
 *
 *  A3 A2 A1 A0
 *  -- -- -- --
 *   0  0  p  0 : Data register (I/O) - p denotes which port - 0=A 1=B
 *   0  0  p  1 : Data Direction Register (I/O) 0=input 1=output
 *   x  1  s  s : Read/Write timer
 *
 *      ss=timer scaling factor - 00=1:1  01=8:1  10=64:1  11=1024:1
 *      x=0 disables IRQ on PB7, x=1 enables (on both read and write)
 *
 *  Timer counts down from the written value on each clock/scale cycle.
 *  When the timer reaches 0, and wraps to FF, the timer "expires" and
 *  if interrupts are enabled, then one is generated. Now the scale
 *  factor is ignored, and the timer decrements on every clock cycle.
 *  At the next timer read, the original timer value and scale factor
 *  are restored, and the interrupt (if one was generated) is reset.
 *
 *  ==KIM-1 <-> KIM-UNO I/O Pin Mapping==
 *  
 *  For both 6530 and AVR, DDR: 0=input 1=output 
 * 
 *    6530          AVR
 *    port  Ard'o  Port
 *    .pin    Pin  .Bit   Usage
 *   -----  -----  ----   ---------------------
 *   PAD.0    D2   PD.2   segA/colA  [ 0  7   E TTY]
 *   PAD.1    D3   PD.3   segB/colB  [ 1  8   F]
 *   PAD.2    D4   PD.4   segC/colC  [ 2  9  AD]
 *   PAD.3    D5   PD.5   segD/colD  [ 3  A  DA]
 *   PAD.4    D6   PD.6   segE/colE  [ 4  B   +]
 *   PAD.5    D7   PD.7   segF/colF  [ 5  C  GO]
 *   PAD.6    D8   PB.0   segG/colG  [ 6  D  PC]
 *   PAD.7    --   ----   tty/tape i/o  <==--not defined in KIM-UNO
 *   -----    A5   PC.5   segDP [ST RS SST] <==--not defined in KIM-1
 *   
 *   PBD.0    --   ----   tty/tape i/o  <==--not defined in KIM-UNO 
 *  PBD.4-1 
 *    0000    D9   PB.1   row0 [  0   1   2   3   4   5   6  ST ]
 *    0001   D10   PB.2   row1 [  7   8   9   A   B   C   D  RS ]
 *    0010   D11   PB.3   row2 [  E   F  AD  DA   +  GO  PC SST ]
 *    0011   ---   ----   row3 [TTY]    <==--not defined in KIM-UNO
 *    0100   D12   PB.4   digit1
 *    0101   D13   PB.5   digit2
 *    0110    A0   PC.0   digit3
 *    0111    A1   PC.1   digit4
 *    1000    A2   PC.2   digit5
 *    1001    A3   PC.3   digit6
 *    ----    A4   PC.4   digit4.5      <==--not defined in KIM-1
 *   PBD.5    --   ----   tty/tape i/o  <==--not defined in KIM-UNO   
 *   PBD.6    --   ----   configured as chip-select - not usable for data
 *   PBD.7    --   ----   tty/tape i/o  <==--not defined in KIM-UNO   
 */

#define ioTimerSize   128
#define ioTimerStart  0x1700

///////
// 6530 (RIOT -- RAM/ROM, I/O, and Timer) registers -- there are two of these
///////

uint8_t   riot_Data[2][2];      // Port A/B data register
uint8_t   riot_DataDir[2][2];   // Port A/B data direction register (0=input 1=output)
uint8_t   riot_Timer[2];        // Timer value -- reads get this value
uint8_t   riot_TimerSet[2];     // Timer value -- writes set this value
uint16_t  riot_TimerScale[2];   // Timer scale - can be 1, 8, 64 or 1024
boolean   riot_TimerExpired[2]; // Timer state -- independent from IRQ state
boolean   riot_IrqEnabled[2];   // If true, IRQ is asserted when timer rolls over to 0xFF
boolean   riot_IrqAsserted[2];  // The state of the 6530 internal IRQ flag

// 6530 timer scale factors

uint16_t scaleFactors[] = { 1, 8, 64, 1024};   

// These are shortcuts for referring to ports as named in the KIM-1 ROM listings

#define PAD  1][0
#define PADD 1][0
#define PBD  1][1
#define PBDD 1][1

/**
 * The following ROM is specific to the KIM-1. 
 */

uint16_t const ROMsize = 2048;
uint16_t const ROMstart = 0x1800;
const unsigned char memROM[ROMsize] PROGMEM = {
#include "KIM_1_ROM.h"
};

/**
 * Programs that are available to be copied to RAM at startup
 */
 
#include "PROGS.h"
#include "DEC_TEST.h"

////////////////////////////////
// Debugging utility functions//
////////////////////////////////

const char hexDigits[] PROGMEM = "0123456789ABCDEF";

void px2(uint8_t theOctet) {
    char digit;

    digit = pgm_read_byte_near(hexDigits + (theOctet>>4)); 
    Serial.print(digit);
    digit = pgm_read_byte_near(hexDigits + (theOctet&0x0f));
    Serial.print(digit);
}

void px4(uint16_t theWord) {
    uint8_t octet;

    octet = (uint8_t)(theWord>>8);
    px2(octet);
    octet = (uint8_t)(theWord&0xff);
    px2(octet);
}

void pb8(uint8_t theValue, char* LoHi) {
    for (uint8_t ix=7; ix<8; ix--) {
        Serial.print((char) LoHi[((theValue >> ix) & 0x01)]);
    }
}

/**
 * Memory access routines
 */

/** IMPORTANT! *************************************************
 * ALL memory accesses happen via the following two functions. *
 ***************************************************************/

// Avoid overhead of local variable creation by declaring these as global

uint8_t riotSelect, portSelect, registerSelect;
uint8_t rowSelect, thePin;
uint8_t portb, portc, portd;
uint8_t pinb,  pinc,  pind;
uint8_t ddrb,  ddrc,  ddrd;

static void memoryFetch(void) {
    regMAR &= 0x1fff;   /** KIM-1 does not decode the 3 MSBs of the address bus **/
	
    if (regMAR < RAMsize)
        regMDR = memRAM[regMAR];
	
    else if ((regMAR >= ROMstart) && ((regMAR-ROMstart) < ROMsize))
        regMDR = pgm_read_byte_near(memROM + (regMAR - ROMstart));
	
    else if ((regMAR >= auxRAMstart) && ((regMAR-auxRAMstart) < auxRAMsize))
        regMDR = auxRAM[regMAR - auxRAMstart];
	
    else if ((regMAR >= ioTimerStart) && ((regMAR-ioTimerStart) < ioTimerSize)) {
        riotSelect = (regMAR & bit6) >> 6;            // 0=6530-003  1=6530-002
		
		if ((regMAR & 0x000c) == 0) {                 // 0b00xx=I/O port  0bx1xx=timer
		    if (riotSelect == 0) {                    // 6530-003 - i/o ports not emulated
				regMDR = 0xff;
			} else {                                  // 6530-002
		        portSelect = (regMAR & bit1) >> 1;    // 0=PORTA  1=PORTB
                registerSelect = (regMAR & bit0);     // 0=data register  1=data direction register

                // Fetch relevent values -- data OR data direction
                
				if (registerSelect == 0) {            // Data register
				    portb = PINB;
					portc = PINC;
					portd = PIND;
				} else {                              // Data direction register
				    portb = DDRB;
					portc = DDRC;
					portd = DDRD;
				}
               
                // Now extract the values for the requested port
                
			    if (portSelect == 0) {                // KIM Port A
				    regMDR = (portd >> 2) & 0x3f;
                    regMDR |= (portb & bit0) << 6;
                    regMDR |= bit7;

                    // Special case: if row 3 is selected, KIM is testing for TTY jumper
                    if ((riot_Data[PBD] & 0x1E) == 0x06) {
                        if (kimTTY) 
                            regMDR &= ~bit0;
                        else
                            regMDR |= bit0;
                    }
                    
                    if (TRACE_IO && (TRACE_ROM || (regPC < ROMstart))) {
                        EXCLUDE_TIME = micros();
                        Serial.print(F("RRR KIM: PortA ")); pb8(regMDR, "-*");
                        Serial.print(F("   AVR: PortB ")); pb8(portb, "-*");
                        Serial.print(F("  PortC ")); pb8(portc, "-*");
                        Serial.print(F("  PortD ")); pb8(portd, "-*");
                        Serial.println("");
                        delay(TRACE_DELAY);
                        BEGIN_TIME += micros() - EXCLUDE_TIME;
                    }
				} else {                              // KIM Port B
                    // It's entirely possible that this is not correct, but the KIM ROM
                    // never reads Port B, so it mostly doesn't matter -- any application
                    // program that messes with Port B is problematic.
                    
                    regMDR = riot_Data[PBD];
				}
			}
		} else {                                      // Timer reads
            riot_IrqAsserted[riotSelect] = false;
            riot_IrqEnabled[riotSelect] = (regMAR & bit3) != 0;
            if ((regMAR & 0x000F) == 7) {
                regMDR = (riot_TimerExpired[riotSelect]) ? bit7 : 0;
            } else {
                regMDR = riot_Timer[riotSelect];
                riot_Timer[riotSelect] = riot_TimerSet[riotSelect];
                riot_TimerExpired[riotSelect] = false;
            }
		}
    } else {
        regMDR = 0xff;
    }
	
    if (TRACE_MEMORY_ACCESS && (TRACE_ROM || (regPC < ROMstart))) {
        EXCLUDE_TIME = micros();
	    Serial.print(F("Fetch: ")); px4(regMAR); 
		Serial.print(F(": ")); px2(regMDR); 
		Serial.println(""); 
		delay(TRACE_DELAY);
        BEGIN_TIME += micros() - EXCLUDE_TIME;
	}

    clockCycle();
}


static void memoryStore(void) {
    if (TRACE_MEMORY_ACCESS && (TRACE_ROM || (regPC < ROMstart))) {
        EXCLUDE_TIME = micros();
		Serial.print(F("Store: ")); px4(regMAR); 
		Serial.print(F(": ")); px2(regMDR); 
		Serial.println(""); 
        delay(TRACE_DELAY);
        BEGIN_TIME += micros() - EXCLUDE_TIME;
	}
	
    regMAR &= 0x1fff;    // KIM-1 does not decode the 3 MSB of the address bus
    if (regMAR < RAMsize) {
        memRAM[regMAR] = regMDR;
		
    } else if ((regMAR >= auxRAMstart) && ((regMAR-auxRAMstart) < auxRAMsize)) {
        auxRAM[regMAR - auxRAMstart] = regMDR;
		
    } else if ((regMAR >= ioTimerStart) && ((regMAR-ioTimerStart) < ioTimerSize)) {
        riotSelect = (regMAR & bit6) >> 6;        // 0=6530-003  1=6530-002
		
		if ((regMAR & 0x000c) == 0) {             // 0b00xx=I/O port  0bx1xx=timer
		    portSelect = (regMAR & bit1) >> 1;    // 0=PORTA  1=PORTB
			
			if (riotSelect == 1) {                // I/O ports for 6530-003 not emulated
				if (portSelect == 0) {            // Port A
					if ((regMAR & bit0) == 0) {           // Data register A
                        riot_Data[PAD] = regMDR;
				
    				    portd = (PIND | 0xfc) ^ (regMDR << 2);
                        portc = PINC | bit5;
                        portb = (PINB | 0x01) ^ ((regMDR & bit6) >> 6);

                        if (kimSST && digitalRead(A3)) portc &= ~bit5;  // DP in dig 6 says SST on
						
                        PORTD = portd;
                        PORTC = portc;
                        PORTB = portb;
						
                        if (TRACE_IO && (TRACE_ROM || (regPC < ROMstart))) {
                            EXCLUDE_TIME = micros();
                            Serial.print(F("WWW KIM: PortA ")); pb8(regMDR, "-*");
                            Serial.print(F("   AVR: PortB ")); pb8(portb, "-*");
                            Serial.print(F("  PortC ")); pb8(portc, "-*");
                            Serial.print(F("  PortD ")); pb8(portd, "-*");
                            Serial.println("");
                            delay(TRACE_DELAY);
                            BEGIN_TIME += micros() - EXCLUDE_TIME;
                        }
					} else {                              // Data Direction Register A
                        riot_Data[PADD] = regMDR;
						portd = (DDRD & 0x03) | (regMDR << 2);
                        portc = (DDRC & 0xDF) | (regMDR & bit5);  // make segDP direction same as other segments
                        portb = (DDRB & 0xFE) | ((regMDR & bit6) >> 6);
                        DDRD = portd;
                        DDRC = portc;
                        DDRB = portb;
                        if (TRACE_IO && (TRACE_ROM || (regPC < ROMstart))) {
                            EXCLUDE_TIME = micros();
                            Serial.print(F("WWW KIM:  DDRA ")); pb8(regMDR, "v^");
                            Serial.print(F("   AVR:  DDRB ")); pb8(portb, "v^");
                            Serial.print(F("   DDRC ")); pb8(portc, "v^");
                            Serial.print(F("   DDRD ")); pb8(portd, "v^");
                            Serial.println("");
                            delay(TRACE_DELAY);
                            BEGIN_TIME += micros() - EXCLUDE_TIME;
                        }
                        PORTD |= ~portd;
                        PORTC |= ~portc;
                        PORTB |= ~portb;
					}
				} else {                                  // Port B 
                    riot_Data[PBD] = regMDR;
					rowSelect = (regMDR & 0x1e) >> 1;
					thePin = U24_74145[rowSelect];
				    if ((regMAR & bit0) == 0) {           // Data register B
                        riot_Data[PBD] = regMDR;

                        if (TRACE_IO && (TRACE_ROM || (regPC < ROMstart))) {
                            EXCLUDE_TIME = micros();
                            Serial.print(F("WWW KIM: PortB ")); pb8(regMDR, "-*");
                            Serial.print(F("   AVR: Pin ")); 
                            if (thePin != 255) 
                                Serial.print(thePin); 
                            else 
                                Serial.print(F("not"));
                            BEGIN_TIME += micros() - EXCLUDE_TIME;
                        }

                        // Set all digit selects to logic 0
                        
                        PORTB = PINB & (0xff ^ (bit4 | bit5));
                        PORTC = PINC & (0xff ^ (bit0 | bit1 | bit2 | bit3 | bit4));
                        
                        // Set all row selects to logic 1
                        
                        PORTB = PINB | (bit1 | bit2 | bit3);
                        
                        if (thePin != 255) {
                            if (rowSelect < 4) {
                                digitalWrite(thePin, LOW);
                            } else {
                                digitalWrite(thePin, HIGH);
                            }
                        }
                        if (TRACE_IO && (TRACE_ROM || (regPC < ROMstart))) {
                            EXCLUDE_TIME = micros();
                            Serial.println(F(" selected"));
                            delay(TRACE_DELAY);
                            BEGIN_TIME += micros() - EXCLUDE_TIME;
                        }

					} else {    // DDR B
                        riot_Data[PBDD] = regMDR;

                        // Default all PBD lines to output
                        // (The KIM monitor never sets any port B pins as input)
                        
                        DDRB = DDRB | (bit1 | bit2 | bit3 | bit4 | bit5);
                        DDRC = DDRC | (bit0 | bit1 | bit2 | bit3 | bit4);
					}
				}
			}                                     
			
		} else {                                  // Timer
            riot_TimerExpired[riotSelect] = false;
            riot_TimerSet[riotSelect] = regMDR;
            riot_Timer[riotSelect] = regMDR;
			
			// TimerScale is set to (factor - 1), which produces a mask we
			// use when looking at hwCycles: 0x0000, 0x0007, 0x003f, 0x01ff
			// Timer is only decremented when the selected bits are all zero.
			
            riot_TimerScale[riotSelect] = scaleFactors[regMAR & (bit1|bit0)] - 1;
            riot_IrqEnabled[riotSelect] = (regMAR & bit3) >> 3;
            riot_IrqAsserted[riotSelect] = false;
		}
    }

    clockCycle();
}

/**
 * Functions used by the various addressing modes. Assuming you know that regMAR
 * is the Memory Address Register, and regMDR is the Memory Data Register, I think
 * these are self-explanatory.
 */

static void push(uint8_t data) {
    regMAR = regSP--;
    regMAR += 256;
    regMDR = data;
    memoryStore();
}


static uint8_t pop(void) {
    regMAR = ++regSP;
    regMAR += 256;
    memoryFetch();
    return regMDR;
}


static void indirection(void) {
    memoryFetch();
    regALUout = regMDR;
    regMAR++;
    memoryFetch();
    regMAR = regMDR;
    regMAR <<= 8;
    regMAR += regALUout;
}


static void zeroPage(void) {
    regMAR = regPC++;
    memoryFetch();
    regMAR = regMDR;
    regMAR &= pageZeroMask;
}


static void absolute(void) {
    regMAR = regPC++;
    memoryFetch();
    regALUout = regMDR;
    regMAR = regPC++;
    memoryFetch();
    regMAR = regMDR;
    regMAR <<= 8;
    regMAR += regALUout;
}

/**
 * Called ONLY by executeOperation(), as the very first thing.
 */

static void operandFetch(boolean noFetch) {
    switch (pgm_read_byte_near(aMode + regOP)) {
        case amImp:       // implied
            noFetch = true;
            break;
        case amAcc:       // accumulator
            regALUin = regA;
            noFetch = true;
            break;
        case amImm:       // immediate
            regMAR = regPC++;
            break;
        case amZpg:       // zero-page
            zeroPage();
            break;
        case amZpx:       // zero-page, X-indexed
            zeroPage();
            regMAR += regX;
            regMAR &= pageZeroMask;
            break;
        case amZpy:       // zero-page, Y-indexed
            zeroPage();
            regMAR += regY;
            regMAR &= pageZeroMask;
            break;
        case amZxi:       // zero-page X-indexed indirect
            zeroPage();
            regMAR += regX;
            regMAR &= pageZeroMask;
            indirection();
            break;
        case amZyi:       // zero-page indirect, Y-indexed
            zeroPage();
            indirection();
            regMAR += regY;
            break;
        case amAbs:       // absolute
            absolute();
            break;
        case amAbx:       // absolute, X-indexed
            absolute();
            regMAR += regX;
            break;
        case amAby:       // absolute, Y-indexed
            absolute();
            regMAR += regY;
            break;
        case amAbi:       // absolute indirect
            absolute();
            indirection();
            break;
        case amRel:       // relative
            regMAR = regPC++;
            memoryFetch();
            regMAR = regMDR;
            if (regMDR & 0x80) {
                regMAR ^= 0xFF;
                regMAR = regPC - (regMAR + 1);
            } else
                regMAR += regPC;
			PBC = ((regMAR & 0xFF00) != (regPC & 0xFF00)) ? true : false;
            noFetch = true;
            break;
        default:
            break;
    }
    if (!noFetch) {
        memoryFetch();
        regALUin = regMDR;
    }
}

/**
 * Called ONLY by executeOperation(), after the operation has completed. The
 * 16-bit register regALUout is used to set the processor status bits since 
 * (1) it holds the result regardless of where it ends up being put, and
 * (2) we need to look at bits beyond those saved in an 8-bit register.
 */

static void setStatus(void) {
    uint8_t affectedBits;

    affectedBits = pgm_read_byte_near(psBits + opIx - 1);
    if (affectedBits == none) return;
    
    /**
     * The Overflow flag is handled in the ADC and SBC routines. See notes
     * there on why this is necessary.
     */

    if (affectedBits & psZ) {     // Zero flag
        if ((regALUout & 0x00ff) == 0)
            regPS |= psZ;
        else
            regPS &= ~psZ;
    }
    
    if (affectedBits & psN) {     // Negative flag
        if (regALUout & 0x0080)
            regPS |= psN;
        else
            regPS &= ~psN;
    }
    
    if (affectedBits & psC) {     // Carry flag
        if (regALUout & 0x0100)
            regPS |= psC;
        else
            regPS &= ~psC;
    }
}

/**
 * Instructions are executed here. Instructions of special interest:
 *
 *  - ADC & SBC (Decimal mode is gnarly!)
 *  - BRK (increments PC by 2!)
 *  - EMT (emulator interactions)
 *
 * All other instructions are pretty straightforward; though take special note
 * of the storeResult flag (used by monadic operations instructions that affect
 * EITHER a memory location OR the accumulator).
 */

static void executeOperation(void) {
    boolean storeResult = false;
    uint8_t theBit;

    if ((opIx==opSTA) || (opIx==opSTX) || (opIx==opSTY) || (opIx==opJMP) || (opIx==opJSR))
        operandFetch(true);
    else
        operandFetch(false);
        
    if (TRACE_INSTRUCTIONS && (TRACE_ROM || (regPC < ROMstart))) {
        EXCLUDE_TIME = micros();
        Serial.print(F("Execute: ")); 
		if (pgm_read_byte_near(operation + regOP) == opBCC)
			Serial.print(Branches[regOP >> 5]);
		else
			Serial.print(Mnemonics[opIx]); 
        Serial.print(F(" ")); Serial.println(modeIDs[pgm_read_byte_near(aMode + regOP)]);
        delay(TRACE_DELAY); 
        BEGIN_TIME += micros() - EXCLUDE_TIME;
	}
	
    switch (pgm_read_byte_near(operation + regOP)) {
        case opADC:
            carry = regPS & psC;
            regALUout = regA + regALUin + carry;
            regPS &= ~(psN | psV | psZ | psC); 
            regPS |= ((regA ^ regALUout) & (regALUin ^ regALUout) & 0x80) ? psV : 0;
            regPS |= (regALUout & 0x00ff) ? 0 : psZ;
            regPS |= regALUout & psN;
            if (regPS & psD) {
                if (((regA & 0x0F) + (regALUin & 0x0F) + carry) > 9)
                    regALUout += 6;
                if ((regALUout & 0xF0) > 0x90)
                    regALUout += 0x60;
            }
            regPS |= (regALUout & 0x0100) ? psC : 0;
            regA = regALUout & 0x00ff;
            break;
            
        case opAND:
            regALUout = regA & regALUin;
            regA = regALUout;
            break;

        case opASL:
            regALUout = regALUin << 1;
            storeResult = true;
            break;

        case opBCC:
            switch (regOP & 0xD0) {
                case 0x10: theBit = psN; break; 
                case 0x50: theBit = psV; break; 
                case 0x90: theBit = psC; break; 
                case 0xD0: theBit = psZ; break; 
	        }
	        theBit &= regPS;
            if (regOP & 0x20) {
                if (theBit) { 
                    regPC = regMAR; 
                    clockCycle(); 
                    if (PBC) clockCycle(); 
                    PBC = false; 
                }
            } else {
                if (!theBit) { 
                    regPC = regMAR; 
                    clockCycle(); 
                    if (PBC) clockCycle(); 
                    PBC = false; 
                }
            }
            break;

        case opBIT:
            regPS &= ~(psN | psV | psZ);
            regPS |= regALUin & (psN | psV);
            regPS |= (regA & regALUin) ? 0 : psZ;
            break;

        case opBRK:
            // The BRK instruction is documented as a 1-byte opcode,
			// but the saved PC is incremented by 2.
            regPC++;
            hwNMI = true;
            break;

        case opCLC:
            regPS &= ~psC;
            break;

        case opCLD:
            regPS &= ~psD;
            break;

        case opCLI:
            regPS &= ~psI;
            break;

        case opCLV:
            regPS &= ~psV;
            break;

        case opCMP:
            regALUout = regA - regALUin;
            regALUout ^= 0x0100;
            break;

        case opCPX:
            regALUout = regX - regALUin;
            regALUout ^= 0x0100;
            break;

        case opCPY:
            regALUout = regY - regALUin;
            regALUout ^= 0x0100;
            break;

        case opDEC:
            regALUout = regALUin - 1;
            storeResult = true;
            break;

        case opDEX:
            regX--;
            regALUout = regX;
            break;

        case opDEY:
            regY--;
            regALUout = regY;
            break;

        case opEOR:
            regALUout = regA ^ regALUin;
            regA = regALUout;
            break;

        case opINC:
            regALUout = regALUin + 1;
            storeResult = true;
            break;

        case opINX:
            regX++;
            regALUout = regX;
            break;

        case opINY:
            regY++;
            regALUout = regY;
            break;

        case opJSR:
            regALUout = regMAR;
            push((uint8_t)((regPC-1) >> 8));
            push((uint8_t)((regPC-1) & 0x00ff));
            regPC = regALUout;
            break;

        case opJMP:
            regPC = regMAR;
            break;

        case opLDA:
            regA = regALUin;
            regALUout = regA;
            break;

        case opLDX:
            regX = regALUin;
            regALUout = regX;
            break;

        case opLDY:
            regY = regALUin;
            regALUout = regY;
            break;

        case opLSR:
            regALUout = regALUin >> 1;
            regALUout &= 0x007f;
            regALUout |= (regALUin & 0x01) << 8;
            storeResult = true;
            break;

        case opNOP:
            break;

        case opORA:
            regALUout = regA | regALUin;
            regA = regALUout;
            break;

        case opPHA:
            push(regA);
            break;

        case opPHP:
            push(regPS);
            break;

        case opPLA:
            regA = pop();
            break;

        case opPLP:
            regPS = pop();
            regPS |= psB | 0x20;
            break;

        case opROL:
            regALUout = ((uint16_t)regALUin) << 1;
            regALUout |= (regPS & psC);
            storeResult = true;
            break;

        case opROR:
            regALUout = regALUin >> 1;
            regALUout |= (regPS & psC) << 7;
            regALUout |= ((uint16_t)regALUin & 0x0001) << 8;
            storeResult = true;
            break;

        case opRTI:
            regPS = pop();
            regPS |= psB | 0x20;
            regPC = pop() + (pop() * 256);
            break;

        case opRTS:
            regPC = pop() + (pop() * 256) + 1;
            break;

        case opSBC:
            borrow = (regPS & psC) ^ psC;
            regHC = 0;
            regALUout = regA - regALUin - borrow;
            regPS &= ~(psN | psV | psZ | psC); 
            regPS |= ((regA ^ regALUout) & (regALUin ^ regALUout) & 0x80) ? psV : 0;
            regPS |= (regALUout & 0x00ff) ? 0 : psZ;
            regPS |= regALUout & psN;
            if (regPS & psD) {
                if (((regA - borrow) & 0x0F) < (regALUin & 0x0F)) {
                    regALUout -= 0x06;
                    regHC = 1;
                }
                if (((regA & 0xF0) - (regHC << 4)) < (regALUin & 0xF0))
                    regALUout -= 0x60;
            }
            regPS |= (regALUout & 0x0100) ? 0 : psC;
            regA = regALUout & 0x00ff;
            break;
            
        case opSEC:
            regPS |= psC;
            break;

        case opSED:
            regPS |= psD;
            break;

        case opSEI:
            regPS |= psI;
            break;

        case opSTA:
            regALUout = regA;
            storeResult = true;
            break;

        case opSTX:
            regALUout = regX;
            storeResult = true;
            break;

        case opSTY:
            regALUout = regY;
            storeResult = true;
            break;

        case opTAX:
            regX = regA;
            regALUout = regX;
            break;

        case opTAY:
            regY = regA;
            regALUout = regY;
            break;

        case opTSX:
            regX = regSP;
            regALUout = regX;
            break;

        case opTXA:
            regA = regX;
            regALUout = regA;
            break;

        case opTXS:
            regSP = regX;
            break;

        case opTYA:
            regA = regY;
            regALUout = regA;
            break;

        case opEMT:  
            /**
             * This is an instruction for interfacing with the emulator.
             * EMT is a mnemonic for "Emulator Trap".
             */
            switch (regMDR) {
                case 1:  // send regA via serial
                    if (regA != 0)
                        Serial.print((char) regA);
                    break;
                case 2:  // is a byte available on the serial interface?
                    regPS |= psZ;          // default to "zero" - no data available
                    if (Serial.available() > 0)
                        regPS &= ~psZ;     // if data *is* available, reset "zero"
                    break;
                case 3:  // receive regA via serial - a "blocking" call
                    pinMode(A5, OUTPUT);
                    digitalWrite(A5, LOW);
                    DDRC = 0x3f;
                    while (Serial.available() == 0) {
                        for (uint8_t ix=0; ix<6; ix++) {
                            digitalWrite(U24_74145[ix+4], HIGH);
                            delay(1);
                            digitalWrite(U24_74145[ix+4], LOW);
                        }
                    }
                    regA = (uint8_t) Serial.read();
                    break;
                case 4:  // display 7 digits x 8-segments (7 + DP)
                    // A: bitmap DP->7..0<-segA
                    // X: digit selector L->0..6<-R
                    
                    break;
                case 8:  // fetch AVR clock value into memory
                    regMAR = regPC++;
                    memoryFetch();
                    regALUout = regMDR;
                    regMAR = regPC++;
                    memoryFetch();
                    regMAR = (((uint16_t)regMDR) * 256) + regALUout;
                    avrMicros = micros();
                    for (uint8_t ix=0; ix<4; ix++) {
                        regMDR = (uint8_t) avrMicros & 0xFF;
                        memoryStore();
                        regMAR++;
                        avrMicros >>= 8;
                    }
                    break;
                case 9:      // use keyboad & display
                    kimTTY = false;
                    break;
                case 10:     // use serial interface
                    kimTTY = true;
                    break;
                case 11:     // display digits with a gap: XXXX XX
                    for (uint8_t ix=0; ix<4; ix++)
                        U24_74145[ix+4] = pgm_read_byte_near(ledGap + ix);
                    break;
                case 12:     // display digits without a gap: XXXXXX
                    for (uint8_t ix=0; ix<4; ix++)
                        U24_74145[ix+4] = pgm_read_byte_near(ledNoGap +ix);
                    break;
                case 16: 
                    // Slight pause, return with Z=1; turn off digit select line.
                    //                   Special for Blackjack
                    // I consider this a cheat to patch a poorly written program.
                    //              ** REMOVE BEFORE DISTRIBUTION **
                    for (uint8_t ix=0; ix<200; ix++) { hwCycles += PINB & 0x01; }
                    regPS |= psZ;
                    regMDR = 0;
                    regMAR = 0x1742;
                    memoryStore();
                    break;    
                default:
                    // Treat EMT with invalid option as an invalid opcode
                    regPC--;
                    hwNMI = true;
                    break;
            }
	        break;
        opZZZ:  // Invalid opcodes cause trap
            hwNMI = true;
            break;
    }
    setStatus();
    if (storeResult) {
        if (pgm_read_byte_near(aMode + regOP) == amAcc)
            regA = regALUout;
        else {
            regMDR = regALUout;
            memoryStore();
        }
    }
}

/**
 * pollHardware() looks at hardware states and responds to them.
 */

static void pollHardware(void) {
	
	// Check for IRQ from the RIOTs
	
	for (int ix=0; ix<2; ix++) {
		hwIRQ |= riot_IrqAsserted[ix];
	}
	
    if (hwNMI && 
        ((regPC & 0x1FFF) < ROMstart) &&
        (pgm_read_byte_near(operation + regOP) != opRTI)) {
        push((uint8_t)(regPC >> 8));
        push((uint8_t)(regPC & 0x00ff));
        if (regOP == 0)          // if BRK caused this 
            push(regPS);         // leave psB set in pushed PS reg
        else                     // but if the source was not BRK
            push(regPS & ~psB);  // reset the pushed psB flag
        regPS |= psI;
        regMAR = 0xfffa;
        indirection();
        regPC = regMAR;
        hwNMI = false;
    } else if (hwIRQ && ((regPS & psI) == 0)) {
        push((uint8_t)(regPC >> 8));
        push((uint8_t)(regPC & 0x00ff));
        push(regPS);
        regPS |= psI;
        regMAR = 0xfffe;
        indirection();
        regPC = regMAR;
        hwIRQ = false;
    }

}

/**
 * pollHost() checks events and states emanating from the emulator host.
 */

static void pollHost(void) {

    // Save the state of ports B and C
        
    pinb = PINB;
    pinc = PINC;
    portb = PORTB;
    portc = PORTC;
    ddrb = DDRB;
    ddrc = DDRC;

    pinMode(A5, INPUT);
    digitalWrite(A5, HIGH);
    DDRB |= (bit1 | bit2 | bit3);          // Set all rows as output
    PORTB = PINB | (bit1 | bit2 | bit3);   // set all rows high

    // Check for ST
    digitalWrite(9, LOW);
    hwNMI = ((digitalRead(A5) == LOW) || (kimSST == true)) ? true : false;
    // while (digitalRead(A5) == LOW) {}
    digitalWrite(9, HIGH);

    // Check for RS
    digitalWrite(10, LOW);
    hwRST = (digitalRead(A5) == LOW) ? true : false;
    while (digitalRead(A5) == LOW) {}
    digitalWrite(10, HIGH);
    
    // Check for SST
    digitalWrite(11, LOW);
    if (digitalRead(A5) == LOW) kimSST = !kimSST;
    while (digitalRead(A5) == LOW) {}
    digitalWrite(11, HIGH);

    // Restore ports B and C to their original state
    
    DDRB = ddrb;
    DDRC = ddrc;
    PORTB = (pinb & ddrb) | (portb & ~ddrb);
    PORTC = (pinc & ddrc) | (portc & ~ddrc);
}

/**
 * Clock cycles are tracked here
 */
 
static void clockCycle(void) {
    hwCycles++;
    
    for (int ix=0; ix<2; ix++) {              // For both 6530s
    
        // If timer has expired, decrement on every cycle
        // Otherwise, decrement only at the rate determined by TimerScale
        
		if (((hwCycles & riot_TimerScale[ix]) == 0) || 
            (riot_TimerExpired[ix] == true)) riot_Timer[ix]--;
            
        // The timer is considered to have expired when it rolls from 00 to FF
        
		if ((riot_Timer[ix] == 0xff) && (riot_TimerExpired[ix] == false)) {
            riot_TimerExpired[ix] = true;        
			riot_IrqAsserted[ix] = riot_IrqEnabled[ix];
        }
    }
}   
    

/**
 * setVectors() is a convenience for users -- whenever memory is cleared by the
 * emulator, the KIM interrupt vectors are automatically set.
 */

static void setVectors(void) {
    // KIM-specific: set interrupt vectors

    regMDR = 0x00;
    regMAR = 0x17FA;  /* NMIV low */
    memoryStore();
    regMAR = 0x17FE;  /* IRQV low */
    memoryStore();
    regMDR = 0x1c;
    regMAR = 0x17FB;  /* NMIV high */
    memoryStore();
    regMAR = 0x17FF;  /* IRQV high */
    memoryStore();
}

/**
 * zeroRam() is another convenience function that doesn't correspond to any real hw.
 */

void zeroRam(void) {
    regMDR = 0;
    for (regMAR=0; regMAR<RAMsize; regMAR++) memoryStore();
    for (regMAR=auxRAMstart; regMAR<(auxRAMstart+auxRAMsize); regMAR++) memoryStore();
    setVectors();
}

/**
 * pollKey() checks for a pressed key during startup
 */

// This table is used to translate electrical row/column coordinates to
// physical layout enumeration: keys 1-4 on the bottom row, 5-8 on the
// second row, etc. 0 means no key was pressed.

const byte keyCode[] PROGMEM = {  0,  
                                  1, 22,  7,  6,  5,  4,  3,  2, 
                                  8, 23, 14, 13, 12, 11, 10,  9,
                                 15, 24, 19, 21, 20, 18, 17, 16 };

byte pollKey() {
    byte row[3];
    
    DDRB  = 0;                    // Set ALL pins in Ports B & C
    PORTB = 0;                    // to input w/o pull-ups
    DDRC  = 0;
    PORTC = 0;
    DDRD &= 0b00000011;           // PD.0 and PD1 are serial TX & RX
    PORTD = PIND & 0b00000011;    // so leave those alone

    DDRB |=        0b00001110;    // PB.1-3 are row selects - output
    
    for (byte ix=0; ix<3; ix++) {
        PORTB = 0; 
        PORTB = (1 << (ix+1));     // Turn on each row select in turn
        delay(10);
        row[ix] = (PIND & 0xFC) | ((PINC & 0x20) >> 4) | (PINB & 0x01); 
    }

    byte theKey = 0;
    if (row[0] | row[1] | row[2]) {
        for (byte theRow=0; theRow < 3; theRow++) {
            for (byte theBit=0; theBit<8; theBit++) {
                if (row[theRow] & (1 << theBit)) theKey = 1 + (theBit + (8 * theRow)); 
            }
        }
    }
    return pgm_read_byte_near(keyCode+theKey);
} 

/**
 * loadProgram() loads an initial test program
 */

void loadProgram(uint8_t *theProgram, uint16_t theLocation, uint16_t theSize) {
    regMAR = theLocation;
    for (int ix = 0; ix<theSize; ix++) {
        regMDR = pgm_read_byte_near(theProgram + ix);
        memoryStore();
        regMAR++; 
    }
}


void setEntryPoint(uint16_t theAddress) {            
    regMDR = (byte) (theAddress & 0xFF);
    regMAR = 0x00FA;
    memoryStore();
    regMAR++;
    regMDR = (byte) (theAddress >> 8);
    memoryStore();
}


void riotReset(uint8_t device) {
    riot_Data[device][0] = 0;
    riot_DataDir[device][0] = 0;
    riot_Data[device][1] = 0;
    riot_DataDir[device][1] = 0;
    riot_Timer[device] = 0;
    riot_TimerSet[device] = 0;
    riot_TimerScale[device] = 0;
    riot_TimerExpired[device] = false;
    riot_IrqEnabled[device] = false;
    riot_IrqAsserted[device] = false;
}


void mpuInit(void) {
    regA = regX = regY = 0;
    regSP = 0xFD;
    regPS = psB | 0x20;
    regPC = 0;
    
    hwRDY = true;
    hwNMI = hwIRQ = false;
    
    regMAR = regALUout = 0;
    regMDR = regALUin = regOP = 0;
    
    hwCycles = 0;
}


void mpuRun(void) {
    char psbits[8] = "NV-BDIZC";
    char ps[9];

    while (hwRDY) {
        EXECUTION_TIME = micros() - BEGIN_TIME;
        if (TRACE_INSTRUCTION_TIMING) {
            Serial.print(F("Cycles=")); Serial.print(hwCycles);
            Serial.print(F("  uSec=")); Serial.println(EXECUTION_TIME);
        }
        BEGIN_TIME = micros();
        if (hwRST) {
            if (TRACE_HW_LINES) {
                EXCLUDE_TIME = micros();
                Serial.println(F("### RST ###"));
                delay(TRACE_DELAY);
                BEGIN_TIME += micros() - EXCLUDE_TIME;
            }
            hwRST = false;
            riotReset(0);
			riotReset(1);
            mpuInit();
            regPS |= psI;
            regMAR = 0xfffc;
            indirection();
            regPC = regMAR;
        }
        regMAR = regPC++;
        memoryFetch();
        regOP = regMDR;
        opIx = pgm_read_byte_near(operation + regOP);

        executeOperation();
        if (TRACE_REGISTERS && (TRACE_ROM || (regPC < ROMstart))) {
            EXCLUDE_TIME = micros();
            Serial.print(F("A=")); px2(regA);
            Serial.print(F(" X=")); px2(regX);
            Serial.print(F(" Y=")); px2(regY);
            Serial.print(F(" SP=")); px2(regSP);
            for (int ix=0; ix<8; ix++) {
                ps[7-ix] = psbits[7-ix];
                if (0 == (regPS & (1 << ix))) ps[7-ix] += 0x20;
            }
            ps[8] = 0;
            Serial.print(F(" PS=")); Serial.print(ps);
            Serial.print(F(" PC=")); px4(regPC);
            Serial.println("");
            delay(TRACE_DELAY);
            BEGIN_TIME += micros() - EXCLUDE_TIME;
        }

        clockCycle();
        pollHost();
        pollHardware();
    }
}


void setup(void) {
    uint8_t ix;
    
    Serial.begin(57600);

    TRACE_MEMORY_ACCESS = false;
    zeroRam();
    setVectors();

    uint8_t testValue = EEPROM.read(eeaLED);
    if ((testValue==12) || (testValue==13)) {  
        for (ix=0; ix<4; ix++)
            U24_74145[ix+4] = EEPROM.read(ix+eeaLED);
    } else {
        for (ix=0; ix<4; ix++)
            EEPROM.write(ix+eeaLED, U24_74145[ix+4]);
    }

    kimTTY = (EEPROM.read(eeaTTY) == 1) ? true : false;
    
    switch (pollKey()) {
        case 0:      /* no key */
            // do nothing special
            break;
        case 1:      /* 0 key - use keyboad & display */
            kimTTY = false;
            EEPROM.write(eeaTTY, 0);
            break;
        case 2:      /* 1 key - use serial interface */
            kimTTY = true;
            EEPROM.write(eeaTTY, 1);
            break;
        case 3:      /* 2 key - display digits with a gap: XXXX XX */
            for (uint8_t ix=0; ix<4; ix++) {
                U24_74145[ix+4] = pgm_read_byte_near(ledGap + ix);
                EEPROM.write(ix+eeaLED, U24_74145[ix+4]);
            }
            break;
        case 4:      /* 3 key - display digits without a gap: XXXXXX */
            for (uint8_t ix=0; ix<4; ix++) {
                U24_74145[ix+4] = pgm_read_byte_near(ledNoGap + ix);
                EEPROM.write(ix+eeaLED, U24_74145[ix+4]);
            }
            break;
        case 5:      /* 4 key */
            loadProgram(_MICROCHESS_0000, 0x0000, sizeof(_MICROCHESS_0000));
            loadProgram(_MICROCHESS_0070, 0x0070, sizeof(_MICROCHESS_0070));
            loadProgram(_MICROCHESS_0100, 0x0100, sizeof(_MICROCHESS_0100));
            loadProgram(_MICROCHESS_0200, 0x0200, sizeof(_MICROCHESS_0200));
            loadProgram(_MICROCHESS_1780, 0x1780, sizeof(_MICROCHESS_1780));
            setEntryPoint(_MICROCHESS_ENTRY);
            break;
        case 6:      /* 5 key */
            loadProgram(_BLACKJACK, 0x0200, sizeof(_BLACKJACK));
            setEntryPoint(_BLACKJACK_ENTRY);
            break;
        case 7:      /* 6 key */
            loadProgram(_WUMPUS_0000, 0x0000, sizeof(_WUMPUS_0000));
            loadProgram(_WUMPUS_0050, 0x0050, sizeof(_WUMPUS_0050));
            loadProgram(_WUMPUS_0100, 0x0100, sizeof(_WUMPUS_0100));
            loadProgram(_WUMPUS_0200, 0x0200, sizeof(_WUMPUS_0200));
            setEntryPoint(_WUMPUS_ENTRY);
            break;
        case 8:      /* 7 key */
            loadProgram(_ASTEROID, 0x0200, sizeof(_ASTEROID));
            setEntryPoint(_ASTEROID_ENTRY);
            break;
        case 9:      /* 8 key */
            loadProgram(_CLOCK, 0x0300, sizeof(_CLOCK));
            setEntryPoint(_CLOCK_ENTRY);
            break;
        case 10:      /* 9 key */
            loadProgram(_LUNAR_LANDER, 0x0200, sizeof(_LUNAR_LANDER));
            setEntryPoint(_LUNAR_LANDER_ENTRY);
            break;
        case 11:      /* A key */
            loadProgram(_DEC_TEST_0200, 0x0200, sizeof(_DEC_TEST_0200));
            setEntryPoint(_DEC_TEST_ENTRY);
            break;
    }
}

void loop(void) {
    
    // enhanced granularity of traces
    
    TRACE_REGISTERS = false;
    TRACE_INSTRUCTIONS = false;
    TRACE_INSTRUCTION_TIMING = false;
    TRACE_MEMORY_ACCESS = false;
    TRACE_IO = false;
    TRACE_HW_LINES = false;
    TRACE_ROM = false;
    TRACE_DELAY = 0;
    
    mpuInit();
    hwRST = true;
    BEGIN_TIME = micros();
	mpuRun();
}
