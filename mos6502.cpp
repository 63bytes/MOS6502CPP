#include "mos6502.h"
#include <iostream>
#include <stdint.h>
#include <fstream>
#include <filesystem>

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
    (this->*OPData.op_func)((this->*OPData.addr_func)());
}

mos6502::mos6502() {
    RAM[0x0200] = 0x69;
    RAM[0x0201] = 0x50;
    RAM[0x0202] = 0x69;
    RAM[0x0203] = 0x50;
    RAM[0x0204] = 0x69;
    RAM[0x0205] = 0x50;
    RAM[0x0206] = 0x69;
    RAM[0x0207] = 0x50;
    RAM[0x0208] = 0x69;
    RAM[0x0209] = 0x50;
    RAM[0x020A] = 0x69;
    RAM[0x020B] = 0x50;

#define MAKE_OPP(HEX,OP_FUNC, ADDR_FUNC, CYCLES, SIZE)\
    instr_table[HEX] = INSTR(\
    HEX,\
    &mos6502::OP_ ## OP_FUNC,\
    &mos6502::Addr_ ## ADDR_FUNC,\
    CYCLES,\
    SIZE\
    );

    //Illegal opcodes
    for (int i = 0; i < 0xff; i++) {
        instr_table[i] = INSTR(i, &mos6502::OP_ILLEGAL, &mos6502::Addr_IMP, 1, 1);;
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
    //endregion

};

//Addressing mode vars
uint8_t IAL; //indirect address low
uint8_t ADL; //Adress low
uint8_t ADH; //Adress high
uint16_t AD;

uint8_t opVal;

//region Adressing mode handlers
uint8_t* mos6502::Addr_IMM() {
    opVal = fetch();//Does not effect RAM
    return &opVal;//pointer to varible
}

uint8_t* mos6502::Addr_ABS() {
    AD = fetch16;
    return &RAM[AD];//Pointer to RAM value
}

uint8_t* mos6502::Addr_ZP() {
    return &RAM[fetch()];//Pointer to Zero page value
}

uint8_t* mos6502::Addr_ACC() {
    return &A;//Pointer to Accumulator varible
}

uint8_t* mos6502::Addr_IMP() {
    return &opVal;
}

uint8_t* mos6502::Addr_INDX() {//list of pointers
    AD = read16(fetch()+X);//get zero page ADDRESS+X
    return &RAM[AD];
}

uint8_t* mos6502::Addr_INDY() {
    AD = read16(fetch());//get address from zero page
    AD += X;//add X to address
    return &RAM[AD];//get value
}

uint8_t* mos6502::Addr_ZPX() {
    return &RAM[fetch()+X];
}

uint8_t* mos6502::Addr_ABX() {
    return &RAM[fetch16+X];
}

uint8_t* mos6502::Addr_ABY() {
    return &RAM[fetch16+X];
}

uint8_t* mos6502::Addr_REL() {
    opVal = fetch();
    return &opVal;
}

uint8_t* mos6502::Addr_IND() {
    return &RAM[fetch16];
}

uint8_t* mos6502::Addr_ZPY() {
    return &RAM[fetch()+Y];
}
//endregion
//Instructions
void mos6502::OP_ILLEGAL(uint8_t *) {}

uint8_t last;
int result;

//region Arithmetic Unit
void mos6502::OP_LDA(uint8_t* m) {
    A = *m;
    SET_ZERO(A);
    SET_NEGATIVE(A);
}

void mos6502::OP_STA(uint8_t* m) {
    *m = A;

}
void mos6502::OP_ADC(uint8_t* m) {
  //  (D) ? SET_CARRY_DEC(A) : SET_CARRY(A,*m);
    std::cout << "[EXECUTE] ADC" << std::endl;
    D ? SET_CARRY_DEC((A+*m)) : SET_CARRY(A+*m);
    last = A;
    A += *m;
    SET_OVERFLOW(last,A);
    SET_NEGATIVE(A);
    SET_ZERO(A);
}

void mos6502::OP_SBC(uint8_t* m) {
    result = A + ~*m+C;
    A = result;
    SET_FLAG(FLAG_C,(result>=0));
    SET_FLAG(FLAG_V, (result>127 or result<127));
    SET_NEGATIVE(A);
    SET_ZERO(A);
}

void mos6502::OP_AND(uint8_t* m) {
    A &= *m;
    SET_ZERO(A);
    SET_NEGATIVE(A);
}

void mos6502::OP_ORA(uint8_t* m) {
    A |= *m;
    SET_ZERO(A);
    SET_NEGATIVE(A);
}

void mos6502::OP_EOR(uint8_t* m) {
    A ^= *m;
    SET_ZERO(A);
    SET_NEGATIVE(A);
}
//endregion

//region Flags
void mos6502::OP_SEC(uint8_t *m) {
    SET_FLAG(FLAG_C,true);
}

void mos6502::OP_CLC(uint8_t *m) {
    SET_FLAG(FLAG_C,false);
}

void mos6502::OP_SEI(uint8_t *m) {
    SET_FLAG(FLAG_I,true);
}

void mos6502::OP_CLI(uint8_t *m) {
    SET_FLAG(FLAG_I,false);
}

void mos6502::OP_SED(uint8_t *m) {
    SET_FLAG(FLAG_D,true);
}

void mos6502::OP_CLD(uint8_t *m) {
    SET_FLAG(FLAG_D,false);
}

void mos6502::OP_CLV(uint8_t *m) {
    SET_FLAG(FLAG_V,false);
}
//endregion

//region Branch and Test
void mos6502::OP_JMP(uint8_t *m) {
    PC = *m;
}
#define BRANCH PC+=static_cast<int8_t>(*m);

void mos6502::OP_BMI(uint8_t *m) {
    if (N) {BRANCH}
}
void mos6502::OP_BPL(uint8_t *m) {
    if (not N) {BRANCH}
}

void mos6502::OP_BCC(uint8_t *m) {
    if (not C) {BRANCH}
}
void mos6502::OP_BCS(uint8_t *m) {
    if (C) {BRANCH}
}
void mos6502::OP_BEQ(uint8_t *m) {
    if (Z) {BRANCH}
}
void mos6502::OP_BNE(uint8_t *m) {
    if  (not Z) {BRANCH}
}
void mos6502::OP_BVS(uint8_t *m) {
    if (V) {BRANCH}
}
void mos6502::OP_BVC(uint8_t *m) {
    if (not V) {BRANCH}
}

void mos6502::OP_CMP(uint8_t *m) {
    result = A-*m;
    SET_ZERO(result);
    SET_NEGATIVE(result);
    SET_FLAG(FLAG_C,*m<=A);
}

void mos6502::OP_BIT(uint8_t *m) {
    result = A & *m;
    SET_NEGATIVE(*m);
    SET_FLAG(FLAG_V,(*m&FLAG_V)==FLAG_V);
}

//endregion

//region Index Register
void mos6502::OP_LDX(uint8_t *m) {
    X = *m;
    SET_ZERO(X);
    SET_NEGATIVE(X);
}

void mos6502::OP_LDY(uint8_t *m) {
    Y = *m;
    SET_ZERO(Y);
    SET_NEGATIVE(Y);
}

void mos6502::OP_STX(uint8_t *m) {
    *m = X;
}

void mos6502::OP_STY(uint8_t *m) {
    *m = Y;
}

void mos6502::OP_INX(uint8_t *m) {
    X += 1;
    SET_NEGATIVE(X);
    SET_ZERO(X);
}

void mos6502::OP_INY(uint8_t *m) {
    Y += 1;
    SET_NEGATIVE(Y);
    SET_ZERO(Y);
}

void mos6502::OP_DEX(uint8_t *m) {
    X -= 1;
    SET_NEGATIVE(X);
    SET_ZERO(X);
}

void mos6502::OP_DEY(uint8_t *m) {
    Y -= 1;
    SET_NEGATIVE(Y);
    SET_ZERO(Y);
}

void mos6502::OP_CPX(uint8_t *m) {
    result = X-*m;
    SET_ZERO(result);
    SET_NEGATIVE(result);
    SET_FLAG(FLAG_C,X>=*m);
}

void mos6502::OP_CPY(uint8_t *m) {
    result = Y-*m;
    SET_ZERO(result);
    SET_NEGATIVE(result);
    SET_FLAG(FLAG_C,Y>=*m);
}

void mos6502::OP_TAX(uint8_t *m) {
    X = A;
    SET_ZERO(X);
    SET_NEGATIVE(X);
}

void mos6502::OP_TXA(uint8_t *m) {
    A = X;
    SET_ZERO(A);
    SET_NEGATIVE(A);
}

void mos6502::OP_TAY(uint8_t *m) {
    Y = A;
    SET_ZERO(Y);
    SET_NEGATIVE(Y);
}

void mos6502::OP_TYA(uint8_t *m) {
    A = Y;
    SET_ZERO(A);
    SET_NEGATIVE(A);
}
//endregion

//region Stack Processing
void mos6502::OP_JSR(uint8_t *m) {
    STACK_PUSH_16(PC)
    PC = AD;
}

void mos6502::OP_RTS(uint8_t *m) {
    PC = STACK_PULL_16;
}

void mos6502::OP_PHA(uint8_t *m) {
    STACK_PUSH(A);
}

void mos6502::OP_PLA(uint8_t *m) {
    A = STACK_PULL();
}

void mos6502::OP_TXS(uint8_t *m) {
    SP = X;
}

void mos6502::OP_TSX(uint8_t *m) {
    X = SP;
}

void mos6502::OP_PHP(uint8_t *m) {
    STACK_PUSH(P);
}

void mos6502::OP_PLP(uint8_t *m) {
    P = STACK_PULL();
}

//endregion

int main() {
    std::cout << "HI" << std::endl;
    std::cout << std::filesystem::current_path() << std::endl;
    uint8_t buffer[255];
    std::ifstream file("ROMS/test", std::ios::binary);
    file.read(reinterpret_cast<char*>(&buffer),1);
    for (int i=0;i<sizeof(buffer);i++) {
        std::cout << static_cast<int>(buffer[i]) << std::endl;
    }
    return 0;
}