
///////
// Addressing modes:
///////

enum aModes {amImp, amAcc, amImm,    // implied, accumulator, immediate
             amZpg, amZpx, amZpy,    // zero-page, zero-page X-indexed, zero-page Y-indexed
             amZxi,                  // zero-page X-indexed indirect
             amZyi,                  // zero-page indirect Y-indexed
             amAbs, amAbx, amAby,    // absolute, absolute x-indexed, absolute y-indexed
             amAbi, amRel,           // absolute indirect, relative
			 amLast };

char *modeIDs[amLast] = { " ", "A", "imm", 
                          "zp", "zp,x", "zp,y", "(zp,x)", "(zp),y",
                          "abs", "abs,x", "abs,y", "(abs)", 
						  "rel" };

/**
 * This table defines the addressing mode of each possible opcode
 */

uint8_t const aMode[256] PROGMEM = {
amImp, amZxi, amImp, amImp, amImp, amZpg, amZpg, amImp, // 00
amImp, amImm, amAcc, amImp, amImp, amAbs, amAbs, amImm, // 08
amRel, amZyi, amImp, amImp, amImp, amZpx, amZpx, amImp, // 10
amImp, amAby, amImp, amImp, amImp, amAbx, amAbx, amImp, // 18
amAbs, amZxi, amImp, amImp, amZpg, amZpg, amZpg, amImp, // 20
amImp, amImm, amAcc, amImp, amAbs, amAbs, amAbs, amImp, // 28
amRel, amZyi, amImp, amImp, amImp, amZpx, amZpx, amImp, // 30
amImp, amAby, amImp, amImp, amImp, amAbx, amAbx, amImp, // 38

amImp, amZxi, amImp, amImp, amImp, amZpg, amZpg, amImp, // 40
amImp, amImm, amAcc, amImp, amAbs, amAbs, amAbs, amImp, // 48
amRel, amZyi, amImp, amImp, amImp, amZpx, amZpx, amImp, // 50
amImp, amAby, amImp, amImp, amImp, amAbx, amAbx, amImp, // 58
amImp, amZxi, amImp, amImp, amImp, amZpg, amZpg, amImp, // 60
amImp, amImm, amAcc, amImp, amAbi, amAbs, amAbs, amImp, // 68
amRel, amZyi, amImp, amImp, amImp, amZpx, amZpx, amImp, // 70
amImp, amAby, amImp, amImp, amImp, amAbx, amAbx, amImp, // 78

amImp, amZxi, amImp, amImp, amZpg, amZpg, amZpg, amImp, // 80
amImp, amImp, amImp, amImp, amAbs, amAbs, amAbs, amImp, // 88
amRel, amZyi, amImp, amImp, amZpx, amZpx, amZpy, amImp, // 90
amImp, amAby, amImp, amImp, amImp, amAbx, amImp, amImp, // 98
amImm, amZxi, amImm, amImp, amZpg, amZpg, amZpg, amImp, // A0
amImp, amImm, amImp, amImp, amAbs, amAbs, amAbs, amImp, // A8
amRel, amZyi, amImp, amImp, amZpx, amZpx, amZpy, amImp, // B0
amImp, amAby, amImp, amImp, amAbx, amAbx, amAby, amImp, // B8

amImm, amZxi, amImp, amImp, amZpg, amZpg, amZpg, amImp, // C0
amImp, amImm, amImp, amImp, amAbs, amAbs, amAbs, amImp, // C8
amRel, amZyi, amImp, amImp, amImp, amZpx, amZpx, amImp, // D0
amImp, amAby, amImp, amImp, amImp, amAbx, amAbx, amImp, // D8
amImm, amZxi, amImp, amImp, amZpg, amZpg, amZpg, amImp, // E0
amImp, amImm, amImp, amImp, amAbs, amAbs, amAbs, amImp, // E8
amRel, amZyi, amImp, amImp, amImp, amZpx, amZpx, amImp, // F0
amImp, amAby, amImp, amImp, amImp, amAbx, amAbx, amImp  // F8
};


///////
// Operations
///////

/**
 * opZZZ denotes an invalid op-code
 * opEMT denotes "Emulator-Trap"
 */

enum Operations { opZZZ, opADC, opAND, opASL, opBCC, opBIT, opBRK, opCLC,
                  opCLD, opCLI, opCLV, opCMP, opCPX, opCPY, opDEC, opDEX,
                  opDEY, opEOR, opINC, opINX, opINY, opJMP, opJSR, opLDA, 
                  opLDX, opLDY, opLSR, opNOP, opORA, opPHA, opPHP, opPLA, 
                  opPLP, opROL, opROR, opRTI, opRTS, opSBC, opSEC, opSED, 
                  opSEI, opSTA, opSTX, opSTY, opTAX, opTAY, opTSX, opTXA, 
                  opTXS, opTYA, opEMT };

/**
 * This table defines the operation of each possible opcode
 */

uint8_t const operation[256] PROGMEM = {
opBRK, opORA, opZZZ, opZZZ, opZZZ, opORA, opASL, opZZZ, // 00
opPHP, opORA, opASL, opZZZ, opZZZ, opORA, opASL, opEMT, // 08
opBCC, opORA, opZZZ, opZZZ, opZZZ, opORA, opASL, opZZZ, // 10
opCLC, opORA, opZZZ, opZZZ, opZZZ, opORA, opASL, opZZZ, // 18
opJSR, opAND, opZZZ, opZZZ, opBIT, opAND, opROL, opZZZ, // 20
opPLP, opAND, opROL, opZZZ, opBIT, opAND, opROL, opZZZ, // 28
opBCC, opAND, opZZZ, opZZZ, opZZZ, opAND, opROL, opZZZ, // 30
opSEC, opAND, opZZZ, opZZZ, opZZZ, opAND, opROL, opZZZ, // 38

opRTI, opEOR, opZZZ, opZZZ, opZZZ, opEOR, opLSR, opZZZ, // 40
opPHA, opEOR, opLSR, opZZZ, opJMP, opEOR, opLSR, opZZZ, // 48
opBCC, opEOR, opZZZ, opZZZ, opZZZ, opEOR, opLSR, opZZZ, // 50
opCLI, opEOR, opZZZ, opZZZ, opZZZ, opEOR, opLSR, opZZZ, // 58
opRTS, opADC, opZZZ, opZZZ, opZZZ, opADC, opROR, opZZZ, // 60
opPLA, opADC, opROR, opZZZ, opJMP, opADC, opROR, opZZZ, // 68
opBCC, opADC, opZZZ, opZZZ, opZZZ, opADC, opROR, opZZZ, // 70
opSEI, opADC, opZZZ, opZZZ, opZZZ, opADC, opROR, opZZZ, // 78

opZZZ, opSTA, opZZZ, opZZZ, opSTY, opSTA, opSTX, opZZZ, // 80
opDEY, opZZZ, opTXA, opZZZ, opSTY, opSTA, opSTX, opZZZ, // 88
opBCC, opSTA, opZZZ, opZZZ, opSTY, opSTA, opSTX, opZZZ, // 90
opTYA, opSTA, opTXS, opZZZ, opZZZ, opSTA, opZZZ, opZZZ, // 98
opLDY, opLDA, opLDX, opZZZ, opLDY, opLDA, opLDX, opZZZ, // A0
opTAY, opLDA, opTAX, opZZZ, opLDY, opLDA, opLDX, opZZZ, // A8
opBCC, opLDA, opZZZ, opZZZ, opLDY, opLDA, opLDX, opZZZ, // B0
opCLV, opLDA, opTSX, opZZZ, opLDY, opLDA, opLDX, opZZZ, // B8

opCPY, opCMP, opZZZ, opZZZ, opCPY, opCMP, opDEC, opZZZ, // C0
opINY, opCMP, opDEX, opZZZ, opCPY, opCMP, opDEC, opZZZ, // C8
opBCC, opCMP, opZZZ, opZZZ, opZZZ, opCMP, opDEC, opZZZ, // D0
opCLD, opCMP, opZZZ, opZZZ, opZZZ, opCMP, opDEC, opZZZ, // D8
opCPX, opSBC, opZZZ, opZZZ, opCPX, opSBC, opINC, opZZZ, // E0
opINX, opSBC, opNOP, opZZZ, opCPX, opSBC, opINC, opZZZ, // E8
opBCC, opSBC, opZZZ, opZZZ, opZZZ, opSBC, opINC, opZZZ, // F0
opSED, opSBC, opZZZ, opZZZ, opZZZ, opSBC, opINC, opZZZ  // F8
};

/**
 * The following two tables are provided for trace
 */

char *Mnemonics[51] = { "---", "ADC", "AND", "ASL", "bcc", "BIT", "BRK", "CLC",
                        "CLD", "CLI", "CLV", "CMP", "CPX", "CPY", "DEC", "DEX",
                        "DEY", "EOR", "INC", "INX", "INY", "JMP", "JSR", "LDA", 
                        "LDX", "LDY", "LSR", "NOP", "ORA", "PHA", "PHP", "PLA", 
                        "PLP", "ROL", "ROR", "RTI", "RTS", "SBC", "SEC", "SED", 
                        "SEI", "STA", "STX", "STY", "TAX", "TAY", "TSX", "TXA", 
                        "TXS", "TYA", "EMT" };

char *Branches[8] = {"BPL", "BMI", "BVC", "BVS", "BCC", "BCS", "BNE", "BEQ"};

/**
 * These defines are for all the possible combinations of processor status
 * flags that are affected by processor operations.
 */

#define NVZC (psN | psV | psZ |psC)
#define NZ   (psN | psZ)
#define NZC  (psN | psZ | psC)
#define NVZ  (psN | psV | psZ)
#define C    (psC)
#define none 0

/**
 * This table, in order by opcode (i.e. the same order as defined in the enum
 * Operations), indicates what flags are affected by each operation.
 *
 * NOTE!! BIT is shown in this table as not updating status flags. It does, in
 * fact, update NVZ -- just not in the standard way used by every other instruction.
 * This table is used by setStatus() to decide what bits to update for a given 
 * instruction, and a value of 'none' for the BIT instruction prevents setStatus()
 * from messing up the status bits as set by the BIT instruction emulation.
 *
 * Similarly, when the decimal flag is set ADC and SBC have their own peculiar ways
 * of determining the PS flags, so this table casuses setStatus() to bypass these.
 */

uint8_t const psBits[50] PROGMEM = {
    none /* ADC */, NZ   /* AND */, NZC  /* ASL */, none /* BCC */, none /* BIT */,
    none /* BRK */, none /* CLC */, none /* CLD */, none /* CLI */, none /* CLV */,
    NZC  /* CMP */, NZC  /* CPX */, NZC  /* CPY */, NZ   /* DEC */, NZ   /* DEX */,
    NZ   /* DEY */, NZ   /* EOR */, NZ   /* INC */, NZ   /* INX */, NZ   /* INY */,
    none /* JMP */, none /* JSR */, NZ   /* LDA */, NZ   /* LDX */, NZ   /* LDY */,
    NZC  /* LSR */, none /* NOP */, NZ   /* ORA */, none /* PHA */, none /* PHP */,
    none /* PLA */, none /* PLP */, NZC  /* ROL */, NZC  /* ROR */, none /* RTI */,
    none /* RTS */, none /* SBC */, none /* SEC */, none /* SED */, none /* SEI */,
    none /* STA */, none /* STX */, none /* STY */, NZ   /* TAX */, NZ   /* TAY */,
    NZ   /* TSX */, NZ   /* TXA */, none /* TXS */, NZ   /* TYA */, none /* EMT */,
};

