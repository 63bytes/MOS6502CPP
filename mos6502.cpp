#include <cstdint>
#include <iostream>

uint16_t constexpr MEMORY_MAX = 0xffff;
int constexpr MICRO_CYCLES = 8;
uint8_t constexpr SIGN_BIT = 0b10000000;
uint8_t constexpr STACK_START = 0xFF;
uint16_t constexpr STACK_PAGE = 0x0100;
uint8_t constexpr HIGH = 0xff00;
uint8_t constexpr LOW = 0x00ff;
uint16_t constexpr BREAK_ADDR = 0xFFFE;
uint8_t constexpr LSB = 0b00000001;

uint8_t signedsub8(uint8_t a, uint8_t b) {
	return a + (~b + 1);
}

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



class mos6502 {
public:
    static void reset() {
        uint8_t RAM[MEMORY_MAX] = {};
    }
    void test() {
        //(this->*OPCODEDATA[0x69].func)(21);
    }
    void Instruction() {
        preExecute();
        execute();
    }
private:
	struct INSTRUCTION_OPHAND{
		uint8_t value;
		uint16_t address;
		bool hasValue=true;
		bool hasAddress=true;
	};
	using OPCODE_FUNCTION = void(mos6502::*)(INSTRUCTION_OPHAND);
	struct Opcode {
		OPCODE_FUNCTION func;
		uint8_t opcode;
		AddressingModes addressing_mode;
		int cycles;
		int bytes;
		Opcode(OPCODE_FUNCTION a, uint8_t b, AddressingModes c, int d, int e){func = a;opcode = b;addressing_mode=c;cycles=d;bytes=e;}
		Opcode(){}
	};
    uint8_t RAM[MEMORY_MAX] = {};

    uint8_t A = 0;
    uint8_t X = 0;
    uint8_t Y = 0;
    uint16_t PC = 0;
    uint8_t S = STACK_START;
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

	Opcode currentOP;
    Opcode OPCODEDATA[0xff] = {};

    uint8_t fetch(const int Increment = 1) {
    	PC += Increment;
        return RAM[PC - Increment];
    }

    uint8_t read(uint16_t address) {
        return RAM[address];
    }

	void stack_push8(uint8_t value) {
	    RAM[STACK_PAGE | S] = value;S--;
    }
	uint8_t stack_pull8(){
		S++;
		return RAM[STACK_PAGE | S];
	}
	void stack_push16(uint16_t value) {
	    stack_push8(HIGH & value);
    	stack_push8(LOW & value);
    }
	uint16_t stack_pulll6() {
		return (stack_pull8()<<8) | stack_pull8();
	}

    uint8_t IAL;//indirect address low
    uint8_t ADL;//Adress low
    uint8_t ADH;//Adress high
    uint8_t BAL;//base addresss
    uint8_t BAH;
    uint8_t OFFSET;
	uint16_t EFA;//effective address
	INSTRUCTION_OPHAND DATA;

    void preExecute() {
        OP = fetch();
		currentOP = OPCODEDATA[OP];
    	DATA.value = 0;
    	DATA.hasValue = true;
    	DATA.address = 0;
    	DATA.hasAddress = true;
        switch (currentOP.addressing_mode) {
            case IMM:
                DATA.value = fetch();
        		DATA.hasAddress = false;
                break;
            case ABS:
                ADL = fetch();
                ADH = fetch();
                DATA.address = (ADH<<8) | ADL;
                break;
            case ZP:
                ADL = fetch();
                DATA.address = ADL;
                break;
            case ACC://No logic needed
        		DATA.hasAddress = false;
                break;
            case IMP:
        		DATA.hasAddress = false;
                break;
            case INDX://List of Pointers (ZP)
                IAL = fetch();
                IAL += X;//Index zero page with X
                ADL = read(IAL);
                ADH = read(IAL+1);
                DATA.address = (ADH<<8) | ADL;
            case INDY://Pointer to a list (ZP)
                IAL = fetch();
                BAL = read(IAL);
                BAH = read(IAL+1);
                DATA.address = (ADH<<8) | ADL;
            case ZPX://Zero page list
                BAL = fetch();
                BAL += X;
                DATA.address = BAL;
            case ABX:
                BAL = fetch() + X;
                BAH = fetch();
                DATA.address = (BAH<<8) | BAL;
            case ABY:
                BAL = fetch() + Y;
                BAH = fetch();
                DATA.address = (BAH<<8) | BAL;
            case REL:
                OFFSET = fetch();
                DATA.value = OFFSET;
        		DATA.address = false;
            case IND:
                IAL = fetch();
                ADL = read(IAL);
                ADH = read(IAL+1);
                DATA.address = (ADH<<8) | ADL;
            case ZPY:
                BAL = fetch();
                BAL += X;
                DATA.address = BAL;
        }
    	if (DATA.hasAddress) {
    		DATA.value = RAM[DATA.address];
    	}
    }

    void execute() {
        (this->*OPCODEDATA[OP].func)(DATA);
    }
    uint8_t result;
	uint8_t tmp8;
    void OP_ADC(const INSTRUCTION_OPHAND d) {
		const uint8_t n = d.value;
        const int s = A & SIGN_BIT;
        result = A + n;
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
	void OP_AND(const INSTRUCTION_OPHAND d) {
		const uint8_t n = d.value;
		A &= n;
		Z = A==0;
		N = (A&SIGN_BIT) == SIGN_BIT;
	}
	void OP_ASL(const INSTRUCTION_OPHAND d) {
		if (OPCODEDATA[OP].addressing_mode==ACC){
			C = (N&SIGN_BIT)==SIGN_BIT;
			A << 1;
			N = (N&SIGN_BIT)==SIGN_BIT;
			Z = N==0;
		}
		else {
			C = (RAM[d.address]&SIGN_BIT)==SIGN_BIT;
			RAM[d.address] << 1;
			N = (RAM[d.address]&SIGN_BIT)==SIGN_BIT;
			Z = RAM[d.address]==0;
		};
	};
	void OP_BCC(const INSTRUCTION_OPHAND d) {
		if (C==false) {
			PC += d.value;//brach if carry reset
		}
	};
	void OP_BCS(const INSTRUCTION_OPHAND d) {
		if (C) {
			PC += d.value;//brach if carry reset
		}
	};
	void OP_BEQ(const INSTRUCTION_OPHAND d) {
		if (Z) {
			PC += d.value;
		}
	}
	void OP_BIT(const INSTRUCTION_OPHAND d) {
		const uint8_t n =  RAM[d.value];
		N = (n&SIGN_BIT)==SIGN_BIT;
		V = (n&0b01000000)==0b01000000;
		result = A & n;
		Z = result==0;
	}
    void OP_BMI(const INSTRUCTION_OPHAND d) {
	    if (N) {
	        PC += d.value;
	    }
	}
    void OP_BNE(const INSTRUCTION_OPHAND d) {
	    if (Z==false) {
	        PC += d.value;
	    }
	}
    void OP_BPL(const INSTRUCTION_OPHAND d) {
	    if (N==false) {
	        PC += d.value;
	    }
	}
    void OP_BRK(const INSTRUCTION_OPHAND d) {
		PC += 1;
		stack_push16(PC);
		stack_push8(P);
		ADH = RAM[BREAK_ADDR];
		ADL = RAM[BREAK_ADDR+1];
		PC = (ADH<<8) | ADL;
		B = true;
	}
    void OP_BVC(const INSTRUCTION_OPHAND d) {
		if (V==false) {
			PC += d.value;
		}
	}
    void OP_BVS(const INSTRUCTION_OPHAND d) {
		if (V) {
			PC += d.value;
		}
	}
    void OP_CLC(const INSTRUCTION_OPHAND d) {
		C = false;
	}
    void OP_CLD(const INSTRUCTION_OPHAND d) {
		D = false;
	}
    void OP_CLI(const INSTRUCTION_OPHAND d) {
		I = false;
	}
    void OP_CLV(const INSTRUCTION_OPHAND d) {
		V = false;
	}
    void OP_CMP(const INSTRUCTION_OPHAND d) {
		result = A - d.value;
		Z = (result==0);
		N = (result&SIGN_BIT)==SIGN_BIT;
		C = d.value<=A;

	}
    void OP_CPX(const INSTRUCTION_OPHAND d) {
		result = X - d.value;
		Z = (d.value==X);
		N = (result&SIGN_BIT)==SIGN_BIT;
		C = d.value>=X;
	}
    void OP_CPY(const INSTRUCTION_OPHAND d) {
		result = signedsub8(Y, d.value);
		Z = (d.value==Y);
		N = (result&SIGN_BIT)==SIGN_BIT;
		C = Y>=d.value;
	}
	void OP_DEC(const INSTRUCTION_OPHAND d) {
		RAM[d.address] += (~1 + 1);
	}
	void OP_DEX(const INSTRUCTION_OPHAND d) {
		X -= 1;
		N = (X&SIGN_BIT)==SIGN_BIT;
		Z = X==0;
	}
	void OP_DEY(const INSTRUCTION_OPHAND d) {
		Y -= 1;
		N = (Y&SIGN_BIT)==SIGN_BIT;
		Z = Y==0;
	}
	void OP_EOR(const INSTRUCTION_OPHAND d) {
		A ^= d.value;
		Z = A==0;
		N = (A&SIGN_BIT)==SIGN_BIT;
	}
	void OP_INC(const INSTRUCTION_OPHAND d) {
		RAM[d.address] ++;
		N = (N&SIGN_BIT)==SIGN_BIT;
		Z = N==0;
	}
	void OP_INX(const INSTRUCTION_OPHAND d) {
		X ++;
		N = (X&SIGN_BIT)==SIGN_BIT;
		Z = X==0;
	}
	void OP_INY(const INSTRUCTION_OPHAND d) {
		Y ++;
		N = (Y&SIGN_BIT)==SIGN_BIT;
		Z = Y==0;
	}
	void OP_JMP(const INSTRUCTION_OPHAND d) {
		PC = d.value;
	}
	void OP_JSR(const INSTRUCTION_OPHAND d) {
		stack_push16(PC);
		PC = d.value;
	}
	void OP_LDA(const INSTRUCTION_OPHAND d) {
		A = d.value;
		Z = A==0;
		N = (A&SIGN_BIT)==SIGN_BIT;
	}
	void OP_LDX(const INSTRUCTION_OPHAND d) {
		X = d.value;
		Z = A==0;
		N = (X&SIGN_BIT)==SIGN_BIT;
	}
	void OP_LDY(const INSTRUCTION_OPHAND d) {
		Y = d.value;
		Z = A==0;
		N = (Y&SIGN_BIT)==SIGN_BIT;
	}
	void OP_LSR(const INSTRUCTION_OPHAND d) {
		if (currentOP.addressing_mode==ACC) {
			C = A&LSB;
			A >> 1;
		}
		else {
			C = RAM[d.address]&LSB;
			RAM[d.address] >> 1;
		}
	}
	void OP_NOP(const INSTRUCTION_OPHAND d) {};
	void OP_ORA(const INSTRUCTION_OPHAND d) {
		A |= d.value;
		Z = A==0;
		N = (A&SIGN_BIT)==SIGN_BIT;
	}
	void OP_PHA(const INSTRUCTION_OPHAND d) {
		stack_push16(A);
	};
	void OP_PHP(const INSTRUCTION_OPHAND d) {
		stack_push8(P);
	}
	void OP_PLA(const INSTRUCTION_OPHAND d) {
		A = stack_pulll6();
		N = (A&SIGN_BIT)==SIGN_BIT;
		Z = A==0;
	}
	void OP_PLP(const INSTRUCTION_OPHAND d) {
		P = stack_pull8();
	}
	void OP_ROL(const INSTRUCTION_OPHAND d) {
		if (currentOP.addressing_mode==ACC) {
			if (C){tmp8=1;}else {tmp8=0;};
			C = A&SIGN_BIT;
			A << 1;
			A |= tmp8;
		}
		else {
			if (C){tmp8=1;}else {tmp8=0;};
			C = RAM[d.address]&SIGN_BIT;
			RAM[d.address] << 1;
			RAM[d.address] |= tmp8;
		}
	}
	void OP_ROR(const INSTRUCTION_OPHAND d) {
		if (C){tmp8=SIGN_BIT;}else {tmp8=0;};
		C = A&0b00000001;
		A >> 1;
		A |= tmp8;
	}
	void OP_RTI(const INSTRUCTION_OPHAND d) {
		P = stack_pull8();
		PC = stack_pulll6();
	}
	void OP_RTS(const INSTRUCTION_OPHAND d) {
		PC = stack_pulll6();
	}
	void OP_SBC(const INSTRUCTION_OPHAND d) {
		A -= d.value;
		A -= P&CI;
	}
	void OP_SEC(const INSTRUCTION_OPHAND d) {
		C = true;
	}
	void OP_SED(const INSTRUCTION_OPHAND d) {
		D = true;
	}
	void OP_SEI(const INSTRUCTION_OPHAND d) {

	}
};

mos6502::mos6502() {
	//Data bus, accumulator and arithmetic unit
	//	LDA - Load Accumulator with Memory
	//	ADC - Add Memory with Carry to Accumulator
	OPCODEDATA[0x69] = Opcode(&mos6502::OP_ADC, 0x69, IMM, 2, 2);
	OPCODEDATA[0x6D] = Opcode(&mos6502::OP_ADC, 0x6D, ABS, 4, 3);
	OPCODEDATA[0x65] = Opcode(&mos6502::OP_ADC, 0x65, ZP, 3, 2);
	OPCODEDATA[0x61] = Opcode(&mos6502::OP_ADC, 0x61, INDX, 6, 2);
	OPCODEDATA[0x71] = Opcode(&mos6502::OP_ADC, 0x71, INDY, 5, 2);
	OPCODEDATA[0x75] = Opcode(&mos6502::OP_ADC, 0x75, ZPX, 4, 2);
	OPCODEDATA[0x7D] = Opcode(&mos6502::OP_ADC, 0x7D, ABX, 4, 3);
	OPCODEDATA[0x79] = Opcode(&mos6502::OP_ADC, 0x79, ABY, 4, 3);
}
}