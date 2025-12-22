//
// Created by gilbe on 22/12/2025.
//
#include <stdint.h>

#ifndef MOS6502
#define MOS6502

class mos6502 {
public:
    mos6502() {
        Instr instr;
        #define MAKE_OP_REF(HEX,OP_FUNC, ADDR_FUNC, CYCLES, SIZE)\
            instr.opcode = HEX;\
            instr.op_f = &mos6502::OP_ ## OP_FUNC;\
            instr_table[HEX] = instr;

        instr_table[0] = Instr(0x69, &mos6502::OP_ADC);
    };
private:
    //Registers
    uint8_t A=0;
    uint8_t X=0;
    uint8_t Y=0;

    //Stack pointer
    uint8_t SP=0;

    //Program counter
    uint16_t PC=0;

    typedef void OPCODE_F(uint8_t*);
    using OPCODE_FP = void (mos6502::*)(uint8_t*);

    //Instruction table
    struct INSTR {
        uint8_t opcode;
        OPCODE_FP op_f;
    };
    Instr instr_table[256];

    //Addressing mode handler
    uint8_t* Addr_IMM();
    uint8_t* Addr_ABS();
    uint8_t* Addr_ZP();
    uint8_t* Addr_ACC();
    uint8_t* Addr_IMP();
    uint8_t* Addr_INDX();
    uint8_t* Addr_INDY();
    uint8_t* Addr_ZPX();
    uint8_t* Addr_ABX();
    uint8_t* Addr_ABY();
    uint8_t* Addr_REL();
    uint8_t* Addr_IND();
    uint8_t* Addr_ZPY();

    //opcode functions


    OPCODE_F OP_LDA;
    OPCODE_F OP_STA;
    OPCODE_F OP_ADC;
    OPCODE_F OP_SBC;
    OPCODE_F OP_AND;
    OPCODE_F OP_ORA;
    OPCODE_F OP_EOR;
};


#endif //MOS6502


