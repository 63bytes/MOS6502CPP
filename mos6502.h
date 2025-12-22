//
// Created by gilbe on 22/12/2025.
//
#include <stdint.h>

#ifndef MOS6502
#define MOS6502

class mos6502 {
private:
    //Registers
    uint8_t A;
    uint8_t X;
    uint8_t Y;

    //Stack pointer
    uint8_t SP;

    //Program counter
    uint16_t PC;
};
mos6502::mos6502();


#endif //UNTITLED_MAIN_H


