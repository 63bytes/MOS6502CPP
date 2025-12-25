#include <cstdint>

#ifndef MOS6502
#define MOS6502

constexpr int HIGH = 0xff00;
constexpr int LOW = UINT8_MAX;
constexpr int UINT8_S = 0b10000000;

constexpr int MEMORY_MAX = 0xffff;
constexpr int STACK_PAGE = 0x0100;
constexpr int STACK_START = UINT8_MAX;
constexpr int PROGRAM_START = 0x0200;

constexpr uint8_t FLAG_N = 0b10000000;
constexpr uint8_t FLAG_V = 0b01000000;
constexpr uint8_t FLAG_B = 0b00010000;
constexpr uint8_t FLAG_D = 0b00001000;
constexpr uint8_t FLAG_I = 0b00000100;
constexpr uint8_t FLAG_Z = 0b00000010;
constexpr uint8_t FLAG_C = 0b00000001;

class mos6502 {
public:
    mos6502();
    void execute();
private:
    //Memory
    uint8_t RAM[MEMORY_MAX];

    //Registers
    uint8_t A=0;
    uint8_t X=0;
    uint8_t Y=0;

    //Flags
    uint8_t P=0;
#define N ((P&FLAG_N)==FLAG_N)
#define V ((P&FLAG_V)==FLAG_V)
#define B ((P&FLAG_B)==FLAG_B)
#define D ((P&FLAG_D)==FLAG_D)
#define I ((P&FLAG_I)==FLAG_I)
#define Z ((P&FLAG_Z)==FLAG_Z)
#define C ((P&FLAG_C)==FLAG_C)
    void SET_FLAG(uint8_t f, bool v) {
        (v) ? P|=f : P&~f;
    }
    void SET_CARRY(uint8_t a) {
        SET_FLAG(FLAG_C,((a)>255));
    }
    void SET_CARRY_DEC(uint8_t a) {
        SET_FLAG(FLAG_C,((a)>99));
    }
    void SET_OVERFLOW(uint8_t a, uint8_t b) {
        SET_FLAG(FLAG_V,((a&UINT8_S) != (b&UINT8_S)));
    }
    void SET_NEGATIVE(uint8_t a) {
        SET_FLAG(FLAG_N, ((a&UINT8_S)==UINT8_S));
    }
    void SET_ZERO(int a) {
        SET_FLAG(FLAG_Z,(a==0));
    }

    //Stack pointer
    uint8_t SP=STACK_START;

    //Program counter
    uint16_t PC=PROGRAM_START;

    uint8_t OP=0;

    //Custom types
    typedef void OPCODE_F();
    using OPCODE_FP = void(mos6502::*)();
    using ADDR_FP = void(mos6502::*)();

    //Instruction table
    struct INSTR {
        uint8_t opcode;
        OPCODE_FP op_func;
        ADDR_FP addr_func;
        int cycles;
        int bytes;
    };

    INSTR OPData;
    INSTR instr_table[256];

    //Utility functions
    uint8_t fetch();

    uint8_t STACK_PULL();
    void STACK_PUSH(uint8_t v);
#define STACK_PULL_16 ((STACK_PULL() << 8) | STACK_PULL())
#define STACK_PUSH_16(V)\
    STACK_PUSH(V&HIGH);\
    STACK_PUSH(V&LOW);
#define SIGN_VAL_8(v) (((v&UINT8_S)==UINT8_S) ? ~v+1 : v)

    //Addressing mode handler
    void Addr_IMM();
    void Addr_ABS();
    void Addr_ZP();
    void Addr_ACC();
    void Addr_IMP();
    void Addr_INDX();
    void Addr_INDY();
    void Addr_ZPX();
    void Addr_ABX();
    void Addr_ABY();
    void Addr_REL();
    void Addr_IND();
    void Addr_ZPY();

    //opcode functions
    OPCODE_F OP_ILLEGAL;

    OPCODE_F OP_LDA;
    OPCODE_F OP_STA;
    OPCODE_F OP_ADC;
    OPCODE_F OP_SBC;
    OPCODE_F OP_AND;
    OPCODE_F OP_ORA;
    OPCODE_F OP_EOR;

    OPCODE_F OP_SEC;
    OPCODE_F OP_CLC;
    OPCODE_F OP_SEI;
    OPCODE_F OP_CLI;
    OPCODE_F OP_SED;
    OPCODE_F OP_CLD;
    OPCODE_F OP_CLV;

    OPCODE_F OP_JMP;
    OPCODE_F OP_BMI;
    OPCODE_F OP_BPL;
    OPCODE_F OP_BCC;
    OPCODE_F OP_BCS;
    OPCODE_F OP_BEQ;
    OPCODE_F OP_BNE;
    OPCODE_F OP_BVS;
    OPCODE_F OP_BVC;
    OPCODE_F OP_CMP;
    OPCODE_F OP_BIT;

    OPCODE_F OP_LDX;
    OPCODE_F OP_LDY;
    OPCODE_F OP_STX;
    OPCODE_F OP_STY;
    OPCODE_F OP_INX;
    OPCODE_F OP_INY;
    OPCODE_F OP_DEX;
    OPCODE_F OP_DEY;
    OPCODE_F OP_CPX;
    OPCODE_F OP_CPY;
    OPCODE_F OP_TAX;
    OPCODE_F OP_TXA;
    OPCODE_F OP_TAY;
    OPCODE_F OP_TYA;

    OPCODE_F OP_JSR;
    OPCODE_F OP_RTS;
    OPCODE_F OP_PHA;
    OPCODE_F OP_PLA;
    OPCODE_F OP_TXS;
    OPCODE_F OP_TSX;
    OPCODE_F OP_PHP;
    OPCODE_F OP_PLP;
};

#endif //MOS6502


