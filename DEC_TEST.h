/**
 * Decimal mode test suite by Bruce Clark
 */

#define _DEC_TEST_ENTRY 0x032F

const byte _DEC_TEST_0000[] PROGMEM = {
                               // ; Verify decimal mode behavior
                               // ; Written by Bruce Clark.  This code is public domain.
                               // ; Ref: 6502.org/tutorials/decimal_mode.html
                               // ;
                               // ; Returns:
                               // ;   ERROR = 0 if the test passed
                               // ;   ERROR = 1 if the test failed
                               // ;
                               // ; Variables:
                               // ;   N1 and N2 are the two numbers to be added or subtracted
                               // ;   N1H, N1L, N2H, and N2L are the upper 4 bits and lower 4 bits
                               // ;     of N1 and N2
                               // ;   DA and DNVZC are the actual accumulator and flag results in 
                               // ;     decimal mode
                               // ;   BA and BNVZC are the accumulator and flag results when N1 
                               // ;     and N2 are added or subtracted using binary arithmetic
                               // ;   PA, PNF, PVF, PZF, and PCF are the predicted decimal mode 
                               // ;     accumulator and flag results, calculated using binary
                               // ;     arithmetic
                               // ;
                               // ; This program takes approximately 1 minute at 1 MHz (a few 
                               // ; seconds more on a 65C02 than a 6502 or 65816)
                               // ;
                               //
    /*0000*/                   //          .ORG  0
    /*0000*/                   // ERROR    .DS   1
                               //
    /*0001*/                   // ADDSUB   .DS   1       ; ADC='AA' SBC='55'
    /*0002*/                   // INC      .DS   1       ; Value of C going in
                               //
    /*0003*/                   // N1       .DS   1       ; Addend 1
    /*0004*/                   // N1H      .DS   1       ; Addend 1 high nibble
    /*0005*/                   // N1L      .DS   1       ; Addend 1 low nibble
    /*0006*/                   // N2       .DS   1       ; Addend 2
    /*0007*/                   // N2H      .DS   2       ; Addend 2 high nibble
    /*0009*/                   // N2L      .DS   1       ; Addend 2 low nibble
                               //
    /*000A*/                   // DA       .DS   1       ; Actual A result in decimal mode
    /*000B*/                   // DNVZC    .DS   1       ; Actual PS result in decimal mode
                               //
    /*000C*/                   // BA       .DS   1       ; Binary A result
    /*000D*/                   // BNVZC    .DS   1       ; Binary PS result (NV-BDIZC)
                               //
    /*000E*/                   // PA       .DS   1       ; Predicted accumulator result
    /*000F*/                   // PCF      .DS   1       ; Predicted carry flag result
    /*0010*/                   // PNF      .DS   1       ; Predicted negative flag result
    /*0011*/                   // PVF      .DS   1       ; Predicted overflow flag result
    /*0012*/                   // PZF      .DS   1       ; Predicted zero flag result
};    

const byte _DEC_TEST_0200[] PROGMEM = {
    /*0200*/                   //          .ORG  $200
    /*0200*/ 0xA0, 0x01,       // TEST     LDY   #1      ; init Y (used to loop through CF values)
    /*0202*/ 0x84, 0x00,       //          STY   ERROR   ; store 1 in ERROR until the test passes
    /*0204*/ 0xA9, 0x00,       //          LDA   #0      ; initialize N1 and N2
    /*0206*/ 0x85, 0x03,       //          STA   N1
    /*0208*/ 0x85, 0x06,       //          STA   N2
    /*020A*/ 0x84, 0x02,       // LOOP1    STY   INC
    /*020C*/ 0xA5, 0x06,       //          LDA   N2      ; N2L = N2 & $0F
    /*020E*/ 0x29, 0x0F,       //          AND   #$0F
    /*0210*/ 0x85, 0x09,       //          STA   N2L
    /*0212*/ 0xA5, 0x06,       //          LDA   N2      ; N2H = N2 & $F0
    /*0214*/ 0x29, 0xF0,       //          AND   #$F0
    /*0216*/ 0x85, 0x07,       //          STA   N2H
    /*0218*/ 0x09, 0x0F,       //          ORA   #$0F    ; N2H+1 = (N2 & $F0) + $0F
    /*021A*/ 0x85, 0x08,       //          STA   N2H+1
    /*021C*/ 0xA5, 0x03,       // LOOP2    LDA   N1      ; N1L = N1 & $0F
    /*021E*/ 0x29, 0x0F,       //          AND   #$0F
    /*0220*/ 0x85, 0x05,       //          STA   N1L
    /*0222*/ 0xA5, 0x03,       //          LDA   N1      ; N1H = N1 & $F0
    /*0224*/ 0x29, 0xF0,       //          AND   #$F0
    /*0226*/ 0x85, 0x04,       //          STA   N1H
                               //
    /*0228*/ 0x20, 0x4E, 0x02, //          JSR   ADD
    /*022B*/ 0x20, 0x18, 0x03, //          JSR   A6502
    /*022E*/ 0x20, 0xF3, 0x02, //          JSR   COMPARE
    /*0231*/ 0xD0, 0x1A,       //          BNE   DONE
                               //
    /*0233*/ 0x20, 0x96, 0x02, //          JSR   SUB
    /*0236*/ 0x20, 0x21, 0x03, //          JSR   S6502
    /*0239*/ 0x20, 0xF3, 0x02, //          JSR   COMPARE
    /*023C*/ 0xD0, 0x0F,       //          BNE   DONE
                               //
    /*023E*/ 0xE6, 0x03,       //          INC   N1
    /*0240*/ 0xD0, 0xDA,       //          BNE   LOOP2   ; loop through all 256 values of N1
                               //
    /*0242*/ 0xE6, 0x06,       //          INC   N2
    /*0244*/ 0xD0, 0xC4,       //          BNE   LOOP1   ; loop through all 256 values of N2
                               //
    /*0246*/ 0x88,             //          DEY
    /*0247*/ 0x10, 0xC1,       //          BPL   LOOP1   ; loop through both values of the CF
                               //
    /*0249*/ 0xA9, 0x00,       //          LDA   #0      ; test passed, so store 0 in ERROR
    /*024B*/ 0x85, 0x00,       //          STA   ERROR
    /*024D*/ 0x60,             // DONE     RTS
                               ////////////////////////////////////////////////////////////////
                               // ; Calculate the actual decimal mode accumulator and flags, the
                               // ; accumulator and flag results when N1 is added to N2 using 
                               // ; binary arithmetic, the predicted accumulator result, the 
                               // ; predicted carry flag, and the predicted V flag
                               // ;
    /*024E*/ 0xA9, 0xAA,       // ADD      LDA   #0xAA
    /*0250*/ 0x85, 0x01,       //          STA   ADDSUB
                               //
    /*0252*/ 0xF8,             //          SED           ; decimal mode
    /*0253*/ 0xC0, 0x01,       //          CPY   #1      ; set carry if Y=1, clear carry if Y=0
    /*0255*/ 0xA5, 0x03,       //          LDA   N1
    /*0257*/ 0x65, 0x06,       //          ADC   N2
    /*0259*/ 0x85, 0x0A,       //          STA   DA      ; actual result in decimal mode
    /*025B*/ 0x08,             //          PHP
    /*025C*/ 0x68,             //          PLA
    /*025D*/ 0x85, 0x0B,       //          STA   DNVZC   ; actual flags result in decimal mode
                               //
    /*025F*/ 0xD8,             //          CLD           ; binary mode
    /*0260*/ 0xC0, 0x01,       //          CPY   #1      ; set carry if Y=1, clear carry if Y=0
    /*0262*/ 0xA5, 0x03,       //          LDA   N1
    /*0264*/ 0x65, 0x06,       //          ADC   N2
    /*0266*/ 0x85, 0x0C,       //          STA   BA      ; accumulator result of N1+N2 in binary
    /*0268*/ 0x08,             //          PHP
    /*0269*/ 0x68,             //          PLA
    /*026A*/ 0x85, 0x0D,       //          STA   BNVZC   ; flags result of N1+N2 in binary
                               //
    /*026C*/ 0xC0, 0x01,       //          CPY   #1
    /*026E*/ 0xA5, 0x05,       //          LDA   N1L
    /*0270*/ 0x65, 0x09,       //          ADC   N2L
    /*0272*/ 0xC9, 0x0A,       //          CMP   #$0A
    /*0274*/ 0xA2, 0x00,       //          LDX   #0
    /*0276*/ 0x90, 0x06,       //          BCC   A1
    /*0278*/ 0xE8,             //          INX
    /*0279*/ 0x69, 0x05,       //          ADC   #5      ; add 6 (carry is set)
    /*027B*/ 0x29, 0x0F,       //          AND   #$0F
    /*027D*/ 0x38,             //          SEC
                               //
    /*027E*/ 0x05, 0x04,       // A1       ORA   N1H
                               // ;
                               // ; if N1L + N2L <  $0A, then add N2 & $F0
                               // ; if N1L + N2L >= $0A, then add (N2 & $F0) + $0F + 1 (CF=1)
                               // ;
    /*0280*/ 0x75, 0x07,       //          ADC   N2H,X
    /*0282*/ 0x08,             //          PHP
    /*0283*/ 0xB0, 0x04,       //          BCS   A2
    /*0285*/ 0xC9, 0xA0,       //          CMP   #$A0
    /*0287*/ 0x90, 0x03,       //          BCC   A3
    /*0289*/ 0x69, 0x5F,       // A2       ADC   #$5F    ; add $60 (carry is set)
    /*028B*/ 0x38,             //          SEC
    /*028C*/ 0x85, 0x0E,       // A3       STA   PA      ; predicted accumulator result
    /*028E*/ 0x08,             //          PHP
    /*028F*/ 0x68,             //          PLA
    /*0290*/ 0x85, 0x0F,       //          STA   PCF      ; predicted carry result
    /*0292*/ 0x68,             //          PLA
                               // ;
                               // ; note that all 8 bits of the P register are stored in PVF
                               // ;
    /*0293*/ 0x85, 0x11,       //          STA   PVF      ; predicted V flags
    /*0295*/ 0x60,             //          RTS
                               ///////////////////////////////////////////////////////////////
                               // ; Calculate the actual decimal mode accumulator and flags, and 
                               // ; the accumulator and flag results when N2 is subtracted from N1
                               // ; using binary arithmetic
                               // ;
    /*0296*/ 0xA9, 0x55,       // SUB      LDA   #0x55
    /*0298*/ 0x85, 0x01,       //          STA   ADDSUB
                               //
    /*029A*/ 0xF8,             //          SED           ; decimal mode
    /*029B*/ 0xC0, 0x01,       //          CPY   #1      ; set carry if Y=1, clear carry if Y=0
    /*029D*/ 0xA5, 0x03,       //          LDA   N1
    /*029F*/ 0xE5, 0x06,       //          SBC   N2
    /*02A1*/ 0x85, 0x0A,       //          STA   DA      ; actual accumulator result in dec mode
    /*02A3*/ 0x08,             //          PHP
    /*02A4*/ 0x68,             //          PLA
    /*02A5*/ 0x85, 0x0B,       //          STA   DNVZC   ; actual flags result in decimal mode
                               //
    /*02A7*/ 0xD8,             //          CLD           ; binary mode
    /*02A8*/ 0xC0, 0x01,       //          CPY   #1      ; set carry if Y=1, clear carry if Y=0
    /*02AA*/ 0xA5, 0x03,       //          LDA   N1
    /*02AC*/ 0xE5, 0x06,       //          SBC   N2
    /*02AE*/ 0x85, 0x0C,       //          STA   BA      ; accumulator result of N1-N2 in binary 
    /*02B0*/ 0x08,             //          PHP
    /*02B1*/ 0x68,             //          PLA
    /*02B2*/ 0x85, 0x0D,       //          STA   BNVZC   ; flags result of N1-N2 in binary 
                               //  
    /*02B4*/ 0x60,             //          RTS
                               // ;
                               // ; Calculate the predicted SBC accumulator result for the
                               // ; 6502 and 65816
                               // ;
    /*02B5*/ 0xC0, 0x01,       // SUB1     CPY   #1      ; set carry if Y=1, clear carry if Y=0
    /*02B7*/ 0xA5, 0x05,       //          LDA   N1L
    /*02B9*/ 0xE5, 0x09,       //          SBC   N2L
    /*02BB*/ 0xA2, 0x00,       //          LDX   #0
    /*02BD*/ 0xB0, 0x07,       //          BCS   S11
    /*02BF*/ 0xE8,             //          INX
    /*02C0*/ 0x38,             //          SEC
    /*02C1*/ 0xE9, 0x06,       //          SBC   #6  
    /*02C3*/ 0x29, 0x0F,       //          AND   #$0F
    /*02C5*/ 0x18,             //          CLC
    /*02C6*/ 0x05, 0x04,       // S11      ORA   N1H
                               // ;
                               // ; if N1L - N2L >= 0, then subtract N2 & $F0
                               // ; if N1L - N2L <  0, then subtract (N2 & $F0) + $0F + 1 (CF=0)
                               // ;
    /*02C8*/ 0xF5, 0x07,       //          SBC   N2H,X
    /*02CA*/ 0xB0, 0x03,       //          BCS   S12
    /*02CC*/ 0x38,             //          SEC
    /*02CD*/ 0xE9, 0x60,       //          SBC   #$60
    /*02CF*/ 0x85, 0x0E,       // S12      STA   PA
    /*02D1*/ 0x60,             //          RTS
                               // ;
                               // ; Calculate the predicted SBC accumulator result for the
                               // ; 6502 and 65C02
                               // ;
    /*02D2*/ 0xC0, 0x01,       // SUB2     CPY   #1      ; set carry if Y=1, clear carry if Y=0
    /*02D4*/ 0xA5, 0x05,       //          LDA   N1L
    /*02D6*/ 0xE5, 0x09,       //          SBC   N2L
    /*02D8*/ 0xA2, 0x00,       //          LDX   #0
    /*02DA*/ 0xB0, 0x04,       //          BCS   S21
    /*02DC*/ 0xE8,             //          INX
    /*02DD*/ 0x29, 0x0F,       //          AND   #$0F
    /*02DF*/ 0x18,             //          CLC
    /*02E0*/ 0x05, 0x04,       // S21      ORA   N1H
                               // ;
                               // ; if N1L - N2L >= 0, then subtract N2 & $F0
                               // ; if N1L - N2L <  0, then subtract (N2 & $F0) + $0F + 1 (CF=0)
                               // ;
    /*02E2*/ 0xF5, 0x07,       //          SBC   N2H,X
    /*02E4*/ 0xB0, 0x03,       //          BCS   S22
    /*02E6*/ 0x38,             //          SEC
    /*02E7*/ 0xE9, 0x60,       //          SBC   #$60
    /*02E9*/ 0xE0, 0x00,       // S22      CPX   #0
    /*02EB*/ 0xF0, 0x03,       //          BEQ   S23
    /*02ED*/ 0x38,             //          SEC
    /*02EE*/ 0xE9, 0x06,       //          SBC   #6
    /*02F0*/ 0x85, 0x0E,       // S23      STA   PA      ; predicted accumulator result
    /*02F2*/ 0x60,             //          RTS
                               ////////////////////////////////////////////////////////////////                           
                               // ;
                               // ; Compare accumulator actual results to predicted results
                               // ;
                               // ; Return:
                               // ;   Z flag = 1 (BEQ branch) if same
                               // ;   Z flag = 0 (BNE branch) if different
                               // ;
    /*02F3*/ 0xA5, 0x0A,       // COMPARE  LDA   DA
    /*02F5*/ 0xC5, 0x0E,       //          CMP   PA
    /*02F7*/ 0xD0, 0x1E,       //          BNE   C1
    /*02F9*/ 0xA5, 0x0B,       //          LDA   DNVZC
    /*02FB*/ 0x45, 0x10,       //          EOR   PNF
    /*02FD*/ 0x29, 0x80,       //          AND   #$80    ; mask off N flag
    /*02FF*/ 0xD0, 0x10,       //          BNE   BYP_NVZ ;C1 was 16
    /*0301*/ 0xA5, 0x0B,       //          LDA   DNVZC
    /*0303*/ 0x45, 0x11,       //          EOR   PVF
    /*0305*/ 0x29, 0x40,       //          AND   #$40    ; mask off V flag
    /*0307*/ 0xD0, 0x08,       //          BNE   BYP_NVZ ;C1 was 0E 
    /*0309*/ 0xA5, 0x0B,       //          LDA   DNVZC
    /*030B*/ 0x45, 0x12,       //          EOR   PZF      ; mask off Z flag
    /*030D*/ 0x29, 0x02,       //          AND   #2
    /*030F*/ 0xD0, 0x00,       //          BNE   BYP_NVZ ;C1 was 06
    /*0311*/ 0xA5, 0x0B,       // BYP_NVZ  LDA   DNVZC
    /*0313*/ 0x45, 0x0F,       //          EOR   PCF
    /*0315*/ 0x29, 0x01,       //          AND   #1      ; mask off C flag
    /*0317*/ 0x60,             // C1       RTS
                               ////////////////////////////////////////////////////////////////
                               // ;
                               // ; These routines store the predicted values for ADC and SBC for
                               // ; the 6502 in PA, PCF, PNF, PVF, and PZF
                               // ;
    /*0318*/ 0xA5, 0x11,       // A6502    LDA   PVF
                               // ;
                               // ; since all 8 bits of the P register were stored in PVF, bit 7 of
                               // ; PVF contains the N flag for PNF
                               // ;
    /*031A*/ 0x85, 0x10,       //          STA   PNF
    /*031C*/ 0xA5, 0x0D,       //          LDA   BNVZC
    /*031E*/ 0x85, 0x12,       //          STA   PZF
    /*0320*/ 0x60,             //          RTS
                               //
    /*0321*/ 0x20, 0xB5, 0x02, // S6502    JSR   SUB1
    /*0324*/ 0xA5, 0x0D,       //          LDA   BNVZC
    /*0326*/ 0x85, 0x10,       //          STA   PNF
    /*0328*/ 0x85, 0x11,       //          STA   PVF
    /*032A*/ 0x85, 0x12,       //          STA   PZF
    /*032C*/ 0x85, 0x0F,       //          STA   PCF
    /*032E*/ 0x60,             //          RTS
                               ////////////////////////////////////////////////////////////////
    /*032F*/ 0x20, 0x00, 0x02, // DT_ENTRY JSR   TEST
    /*0332*/ 0x4C, 0x00, 0x1C  //          JMP   $1C00             ; NMI ENTRY
                               //          .END  DT_ENTRY
};
