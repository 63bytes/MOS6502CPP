#include "mos6502.h"
#include <iostream>
#include <stdint.h>
#include <fstream>
#include <filesystem>
#include "font.h"

#define LWHG_CMB(l,h) ((h<<8)|l)

uint8_t mos6502::STACK_PULL() {
    SP++;
    return RAM[SP];
}

void mos6502::STACK_PUSH(uint8_t v) {
    RAM[SP] = v;
    SP--;
}

uint8_t mos6502::fetch() {
    PC++;
    return RAM[PC-1];
}
#define fetch16 ((fetch()<<8) | fetch())
#define read16(a) ((RAM[a]<<8)|RAM[a+1])

void mos6502::execute() {
    OP = fetch();
    OPData = instr_table[OP];
    (this->*OPData.addr_func)();
    (this->*OPData.op_func)();
}

mos6502::mos6502() {
    //region Instruction table
#define MAKE_OPP(HEX,OP_FUNC, ADDR_FUNC, CYCLES, SIZE)\
    instr_table[HEX] = INSTR(\
    HEX,\
    &mos6502::OP_ ## OP_FUNC,\
    &mos6502::Addr_ ## ADDR_FUNC,\
    CYCLES,\
    SIZE\
    );
    instr_table[0xA9] = INSTR(0xA9, &mos6502::OP_LDA, &mos6502::Addr_IMM, 2, 2 );
    //Illegal opcodes
    for (int i = 0; i < 0xff; i++) {
        instr_table[i] = INSTR(i, &mos6502::OP_ILLEGAL, &mos6502::Addr_IMP, 1, 1);
    }

    //region Arith
    //region LDA
    //Data bus, Accumulator, Arithmetic unit
    //LDA - Load Accumulator with Memory, M -> A
    instr_table[0xA9] = INSTR(0xA9, &mos6502::OP_LDA, &mos6502::Addr_IMM, 2, 2 );
    MAKE_OPP(0xAD,LDA,ABS,4,3)
    MAKE_OPP(0xA5,LDA,ZP,3,2)
    MAKE_OPP(0xA1,LDA,INDX,6,2)
    MAKE_OPP(0xB1,LDA,INDY,5,2)
    MAKE_OPP(0xB5,LDA,ZPX,4,2)
    MAKE_OPP(0xBD,LDA,ABX,4,3)
    MAKE_OPP(0xB9,LDA,ABY,4,3)
    //endregion
    //region STA
    //STA - Store Accumulator in Memory, A -> M
    MAKE_OPP(0x8D,STA,ABS,4,3)
    MAKE_OPP(0x85,STA,ZP,3,2)
    MAKE_OPP(0x81,STA,INDX,6,2)
    MAKE_OPP(0x91,STA,INDY,6,2)
    MAKE_OPP(0x95,STA,ZPX,4,2)
    MAKE_OPP(0x9D,STA,ABX,5,3)
    MAKE_OPP(0x99,STA,ABY,5,3)
    //endregion
    //region ADC
    //ADC - Add Memory to Accumulator with Carry, M + A + C -> A
    MAKE_OPP(0x69,ADC,IMM,2,2)
    MAKE_OPP(0x6D,ADC,ABS,4,3)
    MAKE_OPP(0x65,ADC,ZP,3,2)
    MAKE_OPP(0x61,ADC,INDX,6,2)
    MAKE_OPP(0x71,ADC,INDY,5,2)
    MAKE_OPP(0x75,ADC,ZPX,4,2)
    MAKE_OPP(0x7D,ADC,ABX,4,3)
    MAKE_OPP(0x79,ADC,ABY,4,3)
    //endregion
    //region SBC
    //SBC - Subtract Memory from Accumulator with Borrow A-M-_C_ -> A
    MAKE_OPP(0xE9,SBC,IMM,2,2)
    MAKE_OPP(0xE2,SBC,ABS,4,3)
    MAKE_OPP(0xE5,SBC,ZP,3,2)
    MAKE_OPP(0xE1,SBC,INDX,6,2)
    MAKE_OPP(0xF1,SBC,INDY,5,2)
    MAKE_OPP(0xF5,SBC,ZPX,4,2)
    MAKE_OPP(0xFD,SBC,ABX,4,3)
    MAKE_OPP(0xF9,SBC,ABY,4,3)
    //endregion
    //region AND
    //AND--Memory with Accumulator A&M -> A
    MAKE_OPP(0x29,AND,IMM,2,2)
    MAKE_OPP(0x2D,AND,ABS,4,3)
    MAKE_OPP(0x25,AND,ZP,3,2)
    MAKE_OPP(0x21,AND,INDX,6,2)
    MAKE_OPP(0x31,AND,INDY,5,2)
    MAKE_OPP(0x35,AND,ZPX,4,2)
    MAKE_OPP(0x3D,AND,ABX,4,3)
    MAKE_OPP(0x39,AND,ABY,4,3)
    //endregion
    //region ORA
    //ORA Memory with Accumulator
    MAKE_OPP(0x09,ORA,IMM,2,2)
    MAKE_OPP(0x0D,ORA,ABS,4,3)
    MAKE_OPP(0x05,ORA,ZP,3,2)
    MAKE_OPP(0x01,ORA,INDX,6,2)
    MAKE_OPP(0x11,ORA,INDY,5,2)
    MAKE_OPP(0x15,ORA,ZPX,4,2)
    MAKE_OPP(0x1D,ORA,ABX,4,3)
    MAKE_OPP(0x19,ORA,ABY,4,3)
    //endregion
    //region EOR
    //EOR--"Exclusive OR" Memory with Accumulator
    MAKE_OPP(0x49,EOR,IMM,2,2)
    MAKE_OPP(0x4D,EOR,ABS,4,3)
    MAKE_OPP(0x45,EOR,ZP,3,2)
    MAKE_OPP(0x41,EOR,INDX,6,2)
    MAKE_OPP(0x51,EOR,INDY,5,2)
    MAKE_OPP(0x55,EOR,ZPX,4,2)
    MAKE_OPP(0x5D,EOR,ABX,4,3)
    MAKE_OPP(0x59,EOR,ABY,4,3)
    //endregion
    //endregion
    //region Flags
    MAKE_OPP(0x38,SEC,IMP,2,1)//SEC - Set Carry Flag
    MAKE_OPP(0x18,CLC,IMP,2,1)//CLC - Clear Carry Flag
    MAKE_OPP(0x78,SEI,IMP,2,1) //SEI - Set Interrupt Disable
    MAKE_OPP(0x58,CLI,IMP,2,1)//CLI - Clear Interrupt Disable
    MAKE_OPP(0xF8,SED,IMP,2,1)//SED - Set Decimal Mode
    MAKE_OPP(0xD8,CLD,IMP,2,2)//CLD - Clear Decimal Mode
    MAKE_OPP(0x88,CLV,IMP,2,1)//CLV - Clear Overflow Flag
    //endregion
    //region Branch,Test
    //Branch on...
    MAKE_OPP(0x30,BMI,REL,2,2)//BMI ...result minus
    MAKE_OPP(0x10,BPL,REL,2,2)//BPL ...result plus
    MAKE_OPP(0x90,BCC,REL,2,2)//BCC ...carry clear
    MAKE_OPP(0xB0,BCS,REL,2,2)//BCS ...carry set
    MAKE_OPP(0xF0,BEQ,REL,2,2)//BEQ ...result zero
    MAKE_OPP(0xD0,BNE,REL,2,2)//BNE ...result not zero
    MAKE_OPP(0x70,BVS,REL,2,2)//BVS ...overflow set
    MAKE_OPP(0x50,BVS,REL,2,2)//BVC ...overflow clear

    //region CMP
    //Compare Memory with Accumulator
    MAKE_OPP(0xC9,CMP,IMM,2,2)
    MAKE_OPP(0xCD,CMP,ABS,4,3)
    MAKE_OPP(0xC5,CMP,ZP,3,2)
    MAKE_OPP(0xC1,CMP,INDX,6,2)
    MAKE_OPP(0xD1,CMP,INDY,5,2)
    MAKE_OPP(0xD5,CMP,ZPX,4,2)
    MAKE_OPP(0xDD,CMP,ABX,4,3)
    MAKE_OPP(0xD9,CMP,ABY,4,3)
    //endregion
    //region BIT
    //Test Bitis in Memory with Accumulator
    MAKE_OPP(0x2C,BIT,ABS,4,3)
    MAKE_OPP(0x24,BIT,ZP,3,2)
    //endregion
    //endregion
    //region Index Registers
    //region LDX
    //Load index register x from memory
    MAKE_OPP(0xA2,LDX,IMM,2,2)
    MAKE_OPP(0xAE,LDX,ABS,4,3)
    MAKE_OPP(0xA6,LDX,ZP,3,2)
    MAKE_OPP(0xBE,LDX,ABY,4,3)
    MAKE_OPP(0xB6,LDX,ZPY,4,2)
    //endregion
    //region LDY
    //Load index register y from memory
    MAKE_OPP(0xA0,LDY,IMM,2,2)
    MAKE_OPP(0xAC,LDY,ABS,4,3)
    MAKE_OPP(0xA4,LDY,ZP,3,2)
    MAKE_OPP(0xB4,LDY,ZPX,4,2)
    MAKE_OPP(0xBC,LDY,ABX,4,3)
    //endregion
    //region STX
    //Store index register x in memory
    MAKE_OPP(0x8E,STX,ABS,4,3)
    MAKE_OPP(0x86,STX,ZP,3,2)
    MAKE_OPP(0x96,STX,ZPY,4,2)
    //endregion
    //region STY
    //Store index register y in memory
    MAKE_OPP(0x8C,STY,ABS,4,3)
    MAKE_OPP(0x86,STY,ZP,3,2)
    MAKE_OPP(0x94,STY,ZPX,4,2)
    //endregion
    MAKE_OPP(0xE8,INX,IMP,2,1)//INX Increment X
    MAKE_OPP(0xC8,INY,IMP,2,1)//INY Increment Y
    MAKE_OPP(0xCA,DEX,IMP,2,1)//DEX Decrement X
    MAKE_OPP(0x88,DEY,IMP,2,1)//DEY Decrement Y

    //region CPX - Compare X with Memory
    MAKE_OPP(0xE0,CPX,IMM,2,2)//CPX
    MAKE_OPP(0xEC,CPX,ABS,4,3)//CPY
    MAKE_OPP(0xE4,CPX,ZP,3,2)
    //endregion
    //region CPY - Compare Y with Memory
    MAKE_OPP(0xC0,CPY,IMM,2,2)
    MAKE_OPP(0xCC,CPY,ABS,4,3)
    MAKE_OPP(0xC4,CPY,ZP,3,2)
    //endregion

    //TAX, TXA, TAY, TYA,
    MAKE_OPP(0xAA,TAX,IMP,2,1)
    MAKE_OPP(0x8A,TXA,IMP,2,1)
    MAKE_OPP(0xAB,TAY,IMP,2,1)
    MAKE_OPP(0x98,TYA,IMP,2,1)
    //endregion
    //region Stack Processing
    MAKE_OPP(0x20,JSR,ABS,6,3)//JSR
    MAKE_OPP(0x60,RTS,IMP,6,1)//RTS
    MAKE_OPP(0x48,PHA,IMP,3,1)//PHA
    MAKE_OPP(0x68,PLA,IMP,4,1)//PLA
    MAKE_OPP(0x9A,TXS,IMP,2,1)//TXS
    MAKE_OPP(0xBA,TSX,IMP,2,1)//TSX
    MAKE_OPP(0x08,PHP,IMP,3,1)//PHP
    MAKE_OPP(0x28,PLP,IMP,4,1)//PLP
    //endregion
    //region Shifts

    //LSR
    MAKE_OPP(0x4E,LSR,ABS,6,3)
    MAKE_OPP(0xA6,LSR,ZP,5,2)
    MAKE_OPP(0xA4,LSR,ACC,2,1)
    MAKE_OPP(0x56,LSR,ZPX,6,2)
    MAKE_OPP(0x5E,LSR,ABX,7,3)

    //ASL
    MAKE_OPP(0x0E,ASL,ABS,6,3)
    MAKE_OPP(0x06,ASL,ZP,5,2)
    MAKE_OPP(0x0A,ASL,ACC,2,1)
    MAKE_OPP(0x16,ASL,ZPX,6,2)
    MAKE_OPP(0x1E,ASL,ABX,7,3)

    //endregion
    //endregion

    std::cout << std::filesystem::current_path() << std::endl;
    uint8_t buffer[255];
    std::ifstream file("ROMS/test", std::ios::binary);
    file.read(reinterpret_cast<char*>(&buffer),sizeof(buffer));
    for (int i=0;i<sizeof(buffer);i++) {
        RAM[PROGRAM_START+i] = buffer[i];
    }
};

//Addressing mode vars
uint8_t IAL; //indirect address low
uint8_t ADL; //Adress low
uint8_t ADH; //Adress high
uint16_t AD;
uint16_t effectiveAddress;

uint8_t opVal;
uint8_t* operand;

//region Adressing mode handlers
void mos6502::Addr_IMM() {
    opVal = fetch();//Does not effect RAM
    operand = &opVal;//pointer to varible
}

void mos6502::Addr_ABS() {
    AD = fetch16;
    operand = &RAM[AD];//Pointer to RAM value
}

void mos6502::Addr_ZP() {
    operand = &RAM[fetch()];//Pointer to Zero page value
}

void mos6502::Addr_ACC() {
    operand = &A;//Pointer to Accumulator varible
}

void mos6502::Addr_IMP() {
    operand = &opVal;
}

void mos6502::Addr_INDX() {//list of pointers
    AD = read16(fetch()+X);//get zero page ADDRESS+X
    operand = &RAM[AD];
}

void mos6502::Addr_INDY() {
    AD = read16(fetch());//get address from zero page
    AD += X;//add X to address
    operand = &RAM[AD];//get value
}

void mos6502::Addr_ZPX() {
    operand = &RAM[fetch()+X];
}

void mos6502::Addr_ABX() {
    operand = &RAM[fetch16+X];
}

void mos6502::Addr_ABY() {
    operand = &RAM[fetch16+X];
}

void mos6502::Addr_REL() {
    opVal = fetch();
    operand = &opVal;
}

void mos6502::Addr_IND() {
    operand = &RAM[fetch16];
}

void mos6502::Addr_ZPY() {
    operand = &RAM[fetch()+Y];
}
//endregion
//Instructions
void mos6502::OP_ILLEGAL() {}

uint8_t last;
int result;

//region Arithmetic Unit
void mos6502::OP_LDA() {
    A = *operand;
    SET_ZERO(A);
    SET_NEGATIVE(A);
}

void mos6502::OP_STA() {
    *operand = A;
}
void mos6502::OP_ADC() {
  //  (D) ? SET_CARRY_DEC(A) : SET_CARRY(A,*m);
    std::cout << "[EXECUTE] ADC" << std::endl;
    D ? SET_CARRY_DEC((A+*operand)) : SET_CARRY(A+*operand);
    last = A;
    A += *operand;
    SET_OVERFLOW(last,A);
    SET_NEGATIVE(A);
    SET_ZERO(A);
}

void mos6502::OP_SBC() {
    result = A + ~*operand+C;
    A = result;
    SET_FLAG(FLAG_C,(result>=0));
    SET_FLAG(FLAG_V, (result>127 or result<127));
    SET_NEGATIVE(A);
    SET_ZERO(A);
}

void mos6502::OP_AND() {
    A &= *operand;
    SET_ZERO(A);
    SET_NEGATIVE(A);
}

void mos6502::OP_ORA() {
    A |= *operand;
    SET_ZERO(A);
    SET_NEGATIVE(A);
}

void mos6502::OP_EOR() {
    A ^= *operand;
    SET_ZERO(A);
    SET_NEGATIVE(A);
}
//endregion

//region Flags
void mos6502::OP_SEC() {
    SET_FLAG(FLAG_C,true);
}

void mos6502::OP_CLC() {
    SET_FLAG(FLAG_C,false);
}

void mos6502::OP_SEI() {
    SET_FLAG(FLAG_I,true);
}

void mos6502::OP_CLI() {
    SET_FLAG(FLAG_I,false);
}

void mos6502::OP_SED() {
    SET_FLAG(FLAG_D,true);
}

void mos6502::OP_CLD() {
    SET_FLAG(FLAG_D,false);
}

void mos6502::OP_CLV() {
    SET_FLAG(FLAG_V,false);
}
//endregion

//region Branch and Test
void mos6502::OP_JMP() {
    PC = *operand;
}
#define BRANCH PC+=static_cast<int8_t>(*operand);

void mos6502::OP_BMI() {
    if (N) {BRANCH}
}
void mos6502::OP_BPL() {
    if (not N) {BRANCH}
}

void mos6502::OP_BCC() {
    if (not C) {BRANCH}
}
void mos6502::OP_BCS() {
    if (C) {BRANCH}
}
void mos6502::OP_BEQ() {
    if (Z) {BRANCH}
}
void mos6502::OP_BNE() {
    if  (not Z) {BRANCH}
}
void mos6502::OP_BVS() {
    if (V) {BRANCH}
}
void mos6502::OP_BVC() {
    if (not V) {BRANCH}
}

void mos6502::OP_CMP() {
    result = A-*operand;
    SET_ZERO(result);
    SET_NEGATIVE(result);
    SET_FLAG(FLAG_C,*operand<=A);
}

void mos6502::OP_BIT() {
    result = A & *operand;
    SET_NEGATIVE(*operand);
    SET_FLAG(FLAG_V,(*operand&FLAG_V)==FLAG_V);
}

//endregion

//region Index Register
void mos6502::OP_LDX() {
    X = *operand;
    SET_ZERO(X);
    SET_NEGATIVE(X);
}

void mos6502::OP_LDY() {
    Y = *operand;
    SET_ZERO(Y);
    SET_NEGATIVE(Y);
}

void mos6502::OP_STX() {
    *operand = X;
}

void mos6502::OP_STY() {
    *operand = Y;
}

void mos6502::OP_INX() {
    X += 1;
    SET_NEGATIVE(X);
    SET_ZERO(X);
}

void mos6502::OP_INY() {
    Y += 1;
    SET_NEGATIVE(Y);
    SET_ZERO(Y);
}

void mos6502::OP_DEX() {
    X -= 1;
    SET_NEGATIVE(X);
    SET_ZERO(X);
}

void mos6502::OP_DEY() {
    Y -= 1;
    SET_NEGATIVE(Y);
    SET_ZERO(Y);
}

void mos6502::OP_CPX() {
    result = X-*operand;
    SET_ZERO(result);
    SET_NEGATIVE(result);
    SET_FLAG(FLAG_C,X>=*operand);
}

void mos6502::OP_CPY() {
    result = Y-*operand;
    SET_ZERO(result);
    SET_NEGATIVE(result);
    SET_FLAG(FLAG_C,Y>=*operand);
}

void mos6502::OP_TAX() {
    X = A;
    SET_ZERO(X);
    SET_NEGATIVE(X);
}

void mos6502::OP_TXA() {
    A = X;
    SET_ZERO(A);
    SET_NEGATIVE(A);
}

void mos6502::OP_TAY() {
    Y = A;
    SET_ZERO(Y);
    SET_NEGATIVE(Y);
}

void mos6502::OP_TYA() {
    A = Y;
    SET_ZERO(A);
    SET_NEGATIVE(A);
}
//endregion

//region Stack Processing
void mos6502::OP_JSR() {
    STACK_PUSH_16(PC)
    PC = AD;
}

void mos6502::OP_RTS() {
    PC = STACK_PULL_16;
}

void mos6502::OP_PHA() {
    STACK_PUSH(A);
}

void mos6502::OP_PLA() {
    A = STACK_PULL();
}

void mos6502::OP_TXS() {
    SP = X;
}

void mos6502::OP_TSX() {
    X = SP;
}

void mos6502::OP_PHP() {
    STACK_PUSH(P);
}

void mos6502::OP_PLP() {
    P = STACK_PULL();
}

//endregion

//region Shift and Memory modify
void mos6502::OP_LSR() {
    SET_FLAG(FLAG_C,GET_BIT(*operand,0));
    *operand>>=1;
    SET_NEGATIVE(*operand);
    SET_ZERO(*operand);
}

void mos6502::OP_ASL() {
    SET_FLAG(FLAG_C,GET_BIT(*operand,7));
    *operand>>=1;
    SET_NEGATIVE(*operand);
    SET_ZERO(*operand);
}

//endregion

#define FULL_BLOCK "\u2588"
#define SPACE " "

void printTextConsoleLarge(const unsigned char t[8]) {
    for (int x = 0;x<7;x++) {
        for (int y = 8; not (y<0); y--) {
            if ((t[x]&(2<<y))==((2<<y))) {
                std::cout << FULL_BLOCK << FULL_BLOCK;
            } else {
                std::cout << SPACE << SPACE;
            }
        }
        std::cout << std::endl;
    }
}

int main() {
    printTextConsoleLarge(FONT_STORE[3]);
    return 0;
}