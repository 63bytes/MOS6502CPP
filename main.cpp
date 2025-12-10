#include "6502.h"
#include <cstdint>
#include <iostream>

int constexpr MEMORY_MAX = 0xffff;
int constexpr MICRO_CYCLES = 8;
uint8_t constexpr SIGN_BIT = 0b10000000;

class ProgramFlag {
private:
    uint8_t store;
    uint8_t index;
public:
    ProgramFlag(uint8_t& s, uint8_t i) { store = s;index = i; }

    operator bool() const {
        if ((store & index) == 1) { return true; }
        else { return false; }
    }

    ProgramFlag& operator=(bool v) {
        if (v) {
            store |= index;
        }
        else {
            store &= ~index;
        }
        return *this;
    }
};

enum AddressingModes {
    IMM,
    ABS,
    ZP,
    ACC,
    IMP,
    INDX,
    INDY,
    ZPX,
    ABX,
    ABY,
    REL,
    IND,
    ZPY
};

class CPU {
public:
    CPU() {
        InitOps();
    }
    static void reset() {
        uint8_t RAM[MEMORY_MAX] = {};
    }
    void test() {
        (this->*opcodeData[0x69].func)(21);
    }
    void Instruction() {
        preExecute();
        execute();
    }
private:
    uint8_t RAM[MEMORY_MAX] = {};

    uint8_t A = 0;
    uint8_t X = 0;
    uint8_t Y = 0;
    uint16_t PC = 0;
    uint8_t S = 0;
    uint8_t P = 0;

    uint8_t NI = 0b10000000;
    uint8_t VI = 0b01000000;
    uint8_t BI = 0b00010000;
    uint8_t DI = 0b00001000;
    uint8_t II = 0b00000100;
    uint8_t ZI = 0b00000010;
    uint8_t CI = 0b00000001;

    ProgramFlag N = ProgramFlag(S, NI);
    ProgramFlag V = ProgramFlag(S, VI);
    ProgramFlag B = ProgramFlag(S, BI);
    ProgramFlag D = ProgramFlag(S, DI);
    ProgramFlag I = ProgramFlag(S, II);
    ProgramFlag Z = ProgramFlag(S, ZI);
    ProgramFlag C = ProgramFlag(S, CI);

    uint8_t OP = 0;
    uint16_t operand = 0;

    int Cycle = 0;
    using OPCODE_FUNCTION = void(CPU::*)(uint16_t);
    struct Opcode {
        OPCODE_FUNCTION func;
        uint8_t opcode;
        AddressingModes addressing_mode;
        int cycles;
        int bytes;
        Opcode(OPCODE_FUNCTION a, uint8_t b, AddressingModes c, int d, int e){func = a;opcode = b;addressing_mode=c;cycles=d;bytes=e;}
        Opcode(){}
    };
    Opcode opcodeData[0xff] = {};
    void InitOps() {
        opcodeData[0x69] = Opcode(&CPU::OP_ADC, 0x69, IMM, 2, 2);
    }

    uint8_t fetch(const bool Increment = true) {
        if (Increment) {
            PC += 1;
            return RAM[PC - 1];
        }
        else {
            return RAM[PC];
        }
    }

    uint8_t read(uint16_t address) {
        return RAM[address];
    }

    uint8_t IAL;//indirect address low
    uint8_t ADL;//Adress low
    uint8_t ADH;//Adress high
    uint8_t DATA;
    uint8_t BAL;
    uint8_t BAH;
    uint8_t OFFSET;
    void preExecute() {
        OP = fetch();
        switch (opcodeData[OP].addressing_mode) {
            case IMM:
                DATA = fetch();
                break;
            case ABS:
                ADL = fetch();
                ADH = fetch();
                DATA = read((ADH << 8) | ADL);
                break;
            case ZP:
                ADL = fetch();
                DATA = read(ADL);
                break;
            case ACC://No logic needed
                DATA = 0;
                break;
            case IMP:
                DATA = 0;
                break;
            case INDX://List of Pointers (ZP)
                IAL = fetch();
                IAL += X;//Index zero page with X
                ADL = read(IAL);
                ADH = read(IAL+1);
                DATA = read((ADH<<8) | ADL);
            case INDY://Pointer to a list (ZP)
                IAL = fetch();
                BAL = read(IAL);
                BAH = read(IAL+1);
                DATA = read((BAH<<8) | BAL+1);
            case ZPX://Zero page list
                BAL = fetch();
                BAL += X;
                DATA = read(BAL);
            case ABX:
                BAL = fetch() + X;
                BAH = fetch();
                DATA = read((BAH<<8) | BAL);
            case ABY:
                BAL = fetch() + Y;
                BAH = fetch();
                DATA = read((BAH<<8) | BAL);
            case REL:
                OFFSET = fetch();
                DATA = OFFSET;
            case IND:
                IAL = fetch();
                ADL = read(IAL);
                ADH = read(IAL+1);
                DATA = read((ADH<<8) | ADL);
            case ZPY:
                BAL = fetch();
                BAL += X;
                DATA = read(BAL);
        }
    }

    void execute() {
        (this->*opcodeData[OP].func)(DATA);
    }

    void OP_ADC(const uint16_t o) {
        const uint8_t n = o;
        const int s = A & SIGN_BIT;
        uint8_t result = A + n;
        if ((P & D) == D) {
            if ((result & 0x0F) > 0x09) {
                result += 0x06;
            }
            if ((result & 0xF0) > 0x90) {
                result += 0x60;
                C = true;
            }
            else {
                C = false;
            }
            A = result;
        }
        else {
            C = A > 0xff;
        }
        V = ((A & SIGN_BIT) != s) and (s == SIGN_BIT);//If sign bit 1>0 overflow
        N = (A & SIGN_BIT) == SIGN_BIT;//If sign bit==1
        Z = A == 0;
        std::cout << "[EXECUTE] ADC" << std::endl;
    }
};

int main() {
    CPU c;
    return 0;
}