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
        (this->*OPCODEDATA[0x69].func)(21);
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
	Opcode currentOP;
    Opcode OPCODEDATA[0xff] = {};
    void InitOps() {//!!! add json reader https://stackoverflow.com/questions/32205981/reading-json-files-in-c
        OPCODEDATA[0x69] = Opcode(&CPU::OP_ADC, 0x69, IMM, 2, 2);
		OPCODEDATA[0x6D] = Opcode(&CPU::OP_ADC, 0x6D, ABS, 4, 3);
		OPCODEDATA[0x65] = Opcode(&CPU::OP_ADC, 0x65, ZP, 3, 2);
		OPCODEDATA[0x61] = Opcode(&CPU::OP_ADC, 0x61, INDX, 6, 2);
		OPCODEDATA[0x71] = Opcode(&CPU::OP_ADC, 0x71, INDY, 5, 2);
		OPCODEDATA[0x75] = Opcode(&CPU::OP_ADC, 0x75, ZPX, 4, 2);
		OPCODEDATA[0x7D] = Opcode(&CPU::OP_ADC, 0x7D, ABX, 4, 3);
		OPCODEDATA[0x79] = Opcode(&CPU::OP_ADC, 0x79, ABY, 4, 3);
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
    uint8_t BAL;//base addresss
    uint8_t BAH;
    uint8_t OFFSET;
	uint16_t EFA;//effective address
    void preExecute() {
        OP = fetch();
		currentOP = OPCODEDATA[OP];
        switch (currentOP.addressing_mode) {
            case IMM:
                DATA = fetch();
                break;
            case ABS:
                ADL = fetch();
                ADH = fetch();
                DATA = (ADH<<8) | ADL;
                break;
            case ZP:
                ADL = fetch();
                DATA = ADL;
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
                DATA = (ADH<<8) | ADL;
            case INDY://Pointer to a list (ZP)
                IAL = fetch();
                BAL = read(IAL);
                BAH = read(IAL+1);
                DATA = (ADH<<8) | ADL;
            case ZPX://Zero page list
                BAL = fetch();
                BAL += X;
                DATA = BAL;
            case ABX:
                BAL = fetch() + X;
                BAH = fetch();
                DATA = (BAH<<8) | BAL;
            case ABY:
                BAL = fetch() + Y;
                BAH = fetch();
                DATA = (BAH<<8) | BAL;
            case REL:
                OFFSET = fetch();
                DATA = OFFSET;
            case IND:
                IAL = fetch();
                ADL = read(IAL);
                ADH = read(IAL+1);
                DATA = (ADH<<8) | ADL;
            case ZPY:
                BAL = fetch();
                BAL += X;
                DATA = BAL;
        }
    }

    void execute() {
        (this->*OPCODEDATA[OP].func)(DATA);
    }
    uint8_t result;
    void OP_ADC(const uint16_t o) {
		if (currentOP.addressing_mode==(IMM)){
			const uint8_t n = o;
		}
		else {
			const uint8_t n = read(o);//read from effective address
		};
        const int s = A & SIGN_BIT;
        result = A + o;
        if ((P & D) == D) {
            if ((result & 0x0F) > 0x09) {
                result += 0x06;
            };
            if ((result & 0xF0) > 0x90) {
                result += 0x60;
                C = true;
            }
            else {
                C = false;
            };
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
	void OP_AND(const uint8_t o) {
		if (currentOP.addressing_mode==(IMM)){
			const uint8_t n = o;
		}
		else {
			const uint8_t n = read(o);//read from effective address
		};
		A &= o;
		Z = A==0;
		N = (A&SIGN_BIT) == SIGN_BIT;
	}
	void OP_ASL(const uint8_t o) {
		if (OPCODEDATA[OP].addressing_mode==ACC){
			C = (N&SIGN_BIT)==SIGN_BIT;
			A << 1;
			N = (N&SIGN_BIT)==SIGN_BIT;
			Z = N==0;
		}
		else {
			C = (RAM[o]&SIGN_BIT)==SIGN_BIT;
			RAM[o] << 1;
			N = (RAM[o]&SIGN_BIT)==SIGN_BIT;
			Z = RAM[o]==0;
		};
	};
	void OP_BCC(const uint8_t o) {
		if (C==false) {
			PC += o;//brach if carry reset
		}
	};
	void OP_BCS(const uint8_t o) {
		if (C) {
			PC += o;//brach if carry reset
		}
	};
	void OP_BEQ(const uint8_t o) {
		if (Z) {
			PC += o;
		}
	}
	void OP_BIT(const uint8_t o) {
		const uint8_t n =  RAM[o];
		N = (n&SIGN_BIT)==SIGN_BIT;
		V = (n&0b01000000)==0b01000000;
		result = A & n;
		Z = result==0;
	}
};