#include "system.h"

//i8080 is little endian as is x86 and basically every modern architecture
//with that in mind, this emulator won't work on big endian systems

//returns value of register pair (B/D/H or stack pointer)
uint16_t GetPair(System* sys, uint8_t pair)
{
    uint8_t reg1, reg2;

    switch (pair)
    {
        case PB:
            reg1 = RB;
            reg2 = RC;
            break;
        case PD:
            reg1 = RD;
            reg2 = RE;
            break;
        case PH:
            reg1 = RH;
            reg2 = RL;
            break;
        case PSP:
            return sys->SP;
        case PSW:
            return sys->Registers[RA] << 8 | sys->Flags;
    }

    return sys->Registers[reg1] << 8 | sys->Registers[reg2];
}

//sets value of register pair (B/D/H or stack pointer)
void SetPair(System* sys, uint8_t pair, uint16_t value)
{
    uint8_t reg1, reg2;

    switch (pair)
    {
        case PB:
            reg1 = RB;
            reg2 = RC;
            break;
        case PD:
            reg1 = RD;
            reg2 = RE;
            break;
        case PH:
            reg1 = RH;
            reg2 = RL;
            break;
        case PSP:
            sys->SP = value;
            return;
        case PSW:
            sys->Registers[RA] = (value & 0xFF00) >> 8;
            sys->Flags = value & 0x00FF;
        default:
            return;
    }

    sys->Registers[reg1] = (value & 0xFF00) >> 8;
    sys->Registers[reg2] = value & 0x00FF;
}

uint8_t* GetRegOrMem(System* sys, uint8_t reg)
{
    if (reg == 6)
        return sys->Memory + GetPair(sys, PH);

    return sys->Registers + reg;
}

uint8_t CheckCondition(System* sys, uint8_t condition)
{
    switch (condition)
    {
        case CNZ:
            return !(sys->Flags & FZ);
        case CZ:
            return sys->Flags & FZ;
        case CNC:
            return !(sys->Flags & FC);
        case CC:
            return sys->Flags & FC;
        case CNP:
            return !(sys->Flags & FP);
        case CP:
            return sys->Flags & FP;
        case CNS:
            return !(sys->Flags & FS);
        case CS:
            return sys->Flags & FS;
    }

    return 0;
}

uint8_t CountSetBits(uint8_t n)
{
    uint8_t value = 0;

    while (n)
    {
        n &= n - 1;
        value++;
    }

    return value;
}

//checks if there's a carry out of the third bit when a and b are added
uint8_t CheckAuxCarry(uint8_t a, uint8_t b)
{
    uint8_t mask = 1 << 3;
    uint8_t and = a & b;
    uint8_t or = a | b;

    for (int i = 0; i < 4; i++, mask = mask >> 1)
    {
        if (!(mask & or))
            return 0;
        
        if (mask & and)
            return 1;
    }

    return 0;
}

//sets flags for any bitwise logic operation, where n is it's result
void SetFlagsLogic(System* sys, uint8_t n)
{
    sys->Flags &= ~FC;
    sys->Flags &= ~FA;
    sys->Flags = CountSetBits(n) % 2 ? sys->Flags & ~FP : sys->Flags | FP;
    sys->Flags = n                   ? sys->Flags & ~FZ : sys->Flags | FZ;
    sys->Flags = n & 0b10000000      ? sys->Flags | FS  : sys->Flags & ~FS;
}

//returns addition result and sets flags
uint8_t AddWithFlags(System* sys, uint8_t a, uint8_t b)
{
    uint8_t value = a + b;
    sys->Flags = (a > 0 && b > 0xFF - a) ? sys->Flags | FC  : sys->Flags & ~FC;
    sys->Flags = CountSetBits(value) % 2 ? sys->Flags & ~FP : sys->Flags | FP;
    sys->Flags = CheckAuxCarry(a, b)     ? sys->Flags | FA  : sys->Flags & ~FA;
    sys->Flags = value                   ? sys->Flags & ~FZ : sys->Flags | FZ;
    sys->Flags = value & 0b10000000      ? sys->Flags | FS  : sys->Flags & ~FS;
    return value;
}

//return subtraction result and sets flags
uint8_t SubWithFlags(System* sys, uint8_t a, uint8_t b)
{
    b = ~b;

    uint8_t value = AddWithFlags(sys, a, b);
    uint8_t carryFlags = sys->Flags & FC | sys->Flags & FA;
    value = AddWithFlags(sys, value, 1);
    sys->Flags |= carryFlags;

    sys->Flags ^= FC;
    return value;
}

void PushOnStack(System* sys, uint16_t n)
{
    if (sys->SP == 0)
        return;

    *(uint16_t*)(sys->Memory + sys->SP - 2) = n;
    sys->SP -= 2;
}

uint16_t PopFromStack(System* sys)
{
    uint16_t value = *(uint16_t*)(sys->Memory + sys->SP);
    sys->SP += 2;
    return value;
}

#pragma region Instructions

//MOV D,S   01DDDSSS          -       Move register to register
void imov(System* sys, uint8_t* instruction)
{
    uint8_t dest = (*instruction & 0b00111000) >> 3;
    uint8_t source = *instruction & 0b00000111;
    *GetRegOrMem(sys, dest) = *GetRegOrMem(sys, source);
}

//MVI D,#   00DDD110 db       -       Move immediate to register
void imvi(System* sys, uint8_t* instruction)
{
    uint8_t dest = (*instruction & 0b00111000) >> 3;
    *GetRegOrMem(sys, dest) = *(instruction + 1);
}

//LXI RP,#  00RP0001 lb hb    -       Load register pair immediate
void ilxi(System* sys, uint8_t* instruction)
{
    uint8_t pair = (*instruction & 0b00110000) >> 4;
    SetPair(sys, pair, *(uint16_t*)(instruction + 1));
}

//LDA a     00111010 lb hb    -       Load A from memory
void ilda(System* sys, uint8_t* instruction)
{
    uint16_t address = *(uint16_t*)(instruction + 1);
    sys->Registers[RA] = sys->Memory[address];
}

//STA a     00110010 lb hb    -       Store A to memory
void ista(System* sys, uint8_t* instruction)
{
    uint16_t address = *(uint16_t*)(instruction + 1);
    sys->Memory[address] = sys->Registers[RA];
}

//LHLD a    00101010 lb hb    -       Load H:L from memory
void ilhld(System* sys, uint8_t* instruction)
{
    uint16_t address = *(uint16_t*)(instruction + 1);
    sys->Registers[RL] = sys->Memory[address];
    sys->Registers[RH] = sys->Memory[address + 1];
}

//SHLD a    00100010 lb hb    -       Store H:L to memory
void ishld(System* sys, uint8_t* instruction)
{
    uint16_t address = *(uint16_t*)(instruction + 1);
    sys->Memory[address] = sys->Registers[RL];
    sys->Memory[address + 1] = sys->Registers[RH];
}

//LDAX RP   00RP1010 *1       -       Load indirect through RC or DE
void ildax(System* sys, uint8_t* instruction)
{
    uint8_t pair = (*instruction & 0b00010000) >> 4;
    sys->Registers[RA] = sys->Memory[GetPair(sys, pair)];
}

//STAX RP   00RP0010 *1       -       Store indirect through RC or DE
void istax(System* sys, uint8_t* instruction)
{
    uint8_t pair = (*instruction & 0b00010000) >> 4;
    sys->Memory[GetPair(sys, pair)] = sys->Registers[RA];
}

//XCHG      11101011          -       Exchange DE and HL content
void ixchg(System* sys, uint8_t* instruction)
{
    uint16_t ph = GetPair(sys, PH);
    SetPair(sys, PH, GetPair(sys, PD));
    SetPair(sys, PD, ph);
}

//ADD S     10000SSS          ZSPCA   Add register to A
void iadd(System* sys, uint8_t* instruction)
{
    uint8_t reg = *instruction & 0b00000111;
    sys->Registers[RA] = AddWithFlags(sys, sys->Registers[RA], *GetRegOrMem(sys, reg));
}

//ADI #     11000110 db       ZSCPA   Add immediate to A
void iadi(System* sys, uint8_t* instruction)
{
    sys->Registers[RA] = AddWithFlags(sys, sys->Registers[RA], *(instruction + 1));
}

//ADC S     10001SSS          ZSCPA   Add register to A with carry
void iadc(System* sys, uint8_t* instruction)
{
    uint8_t reg = *instruction & 0b00000111;
    sys->Registers[RA] = AddWithFlags(sys, sys->Registers[RA], *GetRegOrMem(sys, reg) + (sys->Flags & FC));
}

//ACI #     11001110 db       ZSCPA   Add immediate to A with carry
void iaci(System* sys, uint8_t* instruction)
{
    sys->Registers[RA] = AddWithFlags(sys, sys->Registers[RA], *(instruction + 1) + (sys->Flags & FC));
}

//SUB S     10010SSS          ZSCPA   Subtract register from A
void isub(System* sys, uint8_t* instruction)
{
    uint8_t source = *instruction & 0b00000111;
    sys->Registers[RA] = SubWithFlags(sys, sys->Registers[RA], *GetRegOrMem(sys, source));
}

//SUI #     11010110 db       ZSCPA   Subtract immediate from A
void isui(System* sys, uint8_t* instruction)
{
    sys->Registers[RA] = SubWithFlags(sys, sys->Registers[RA], *(instruction + 1));
}

//SBB S     10011SSS          ZSCPA   Subtract register from A with borrow
void isbb(System* sys, uint8_t* instruction)
{
    uint8_t reg = *instruction & 0b00000111;
    sys->Registers[RA] = SubWithFlags(sys, sys->Registers[RA], *GetRegOrMem(sys, reg) + (sys->Flags & FC));
}

//SBI #     11011110 db       ZSCPA   Subtract immediate from A with borrow
void isbi(System* sys, uint8_t* instruction)
{
    sys->Registers[RA] = SubWithFlags(sys, sys->Registers[RA], *(instruction + 1) + (sys->Flags & FC));
}

//INR D     00DDD100          ZSPA    Increment register
void iinr(System* sys, uint8_t* instruction)
{
    uint8_t reg = (*instruction & 0b00111000) >> 3;
    sys->Registers[reg] = AddWithFlags(sys, *GetRegOrMem(sys, reg), 1);
}

//DCR D     00DDD101          ZSPA    Decrement register
void idcr(System* sys, uint8_t* instruction)
{
    uint8_t reg = (*instruction & 0b00111000) >> 3;
    sys->Registers[reg] = SubWithFlags(sys, *GetRegOrMem(sys, reg), 1);
}

//INX RP    00RP0011          -       Increment register pair
void iinx(System* sys, uint8_t* instruction)
{
    uint8_t pair = (*instruction & 0b00110000) >> 4;
    SetPair(sys, pair, GetPair(sys, pair) + 1);
}

//DCX RP    00RP1011          -       Decrement register pair
void idcx(System* sys, uint8_t* instruction)
{
    uint8_t pair = (*instruction & 0b00110000) >> 4;
    SetPair(sys, pair, GetPair(sys, pair) - 1);
}

//DAD RP    00RP1001          C       Add register pair to HL (16 bit add)
void idad(System* sys, uint8_t* instruction)
{
    uint8_t pair = (*instruction & 0b00110000) >> 4;
    uint16_t pairValue = GetPair(sys, pair);
    uint16_t hlValue = GetPair(sys, PH);
    sys->Flags = (pairValue > 0 && hlValue > 0xFFFF - pairValue) ? sys->Flags | FC : sys->Flags & ~FC;
    SetPair(sys, PH, pairValue + hlValue);
}

//DAA       00100111          ZSPCA   Decimal Adjust accumulator
void idaa(System* sys, uint8_t* instruction)
{
    uint8_t carry = sys->Flags & FC;
    if (sys->Flags & FA || (sys->Registers[RA] & 0x0F) > 9)
    {
        sys->Registers[RA] = AddWithFlags(sys, sys->Registers[RA], 6);
    }
    sys->Flags |= carry;

    if (sys->Flags & FC || (sys->Registers[RA] & 0xF0) > (9 << 4))
    {
        uint8_t nibble = AddWithFlags(sys, sys->Registers[RA] & 0xF0, 6 << 4); //incrementing only the most significant 4 bits
        sys->Registers[RA] &= 0x0F;
        sys->Registers[RA] |= nibble;
    }
}

//ANA S     10100SSS          ZSCPA   AND register with A
void iana(System* sys, uint8_t* instruction)
{
    uint8_t source = *instruction & 0b00000111;
    sys->Registers[RA] &= *GetRegOrMem(sys, source);
    SetFlagsLogic(sys, sys->Registers[RA]);
}

//ANI #     11100110 db       ZSPCA   AND immediate with A
void iani(System* sys, uint8_t* instruction)
{
    sys->Registers[RA] &= *(instruction + 1);
    SetFlagsLogic(sys, sys->Registers[RA]);
}

//ORA S     10110SSS          ZSPCA   OR  register with A
void iora(System* sys, uint8_t* instruction)
{
    uint8_t source = *instruction & 0b00000111;
    sys->Registers[RA] |= *GetRegOrMem(sys, source);
    SetFlagsLogic(sys, sys->Registers[RA]);
}

//ORI #     11110110          ZSPCA   OR  immediate with A
void iori(System* sys, uint8_t* instruction)
{
    sys->Registers[RA] |= *(instruction + 1);
    SetFlagsLogic(sys, sys->Registers[RA]);
}

//XRA S     10101SSS          ZSPCA   ExclusiveOR register with A
void ixra(System* sys, uint8_t* instruction)
{
    uint8_t source = *instruction & 0b00000111;
    sys->Registers[RA] ^= *GetRegOrMem(sys, source);
    SetFlagsLogic(sys, sys->Registers[RA]);
}

//XRI #     11101110 db       ZSPCA   ExclusiveOR immediate with A
void ixri(System* sys, uint8_t* instruction)
{
    sys->Registers[RA] ^= *(instruction + 1);
    SetFlagsLogic(sys, sys->Registers[RA]);
}

//CMP S     10111SSS          ZSPCA   Compare register with A
void icmp(System* sys, uint8_t* instruction)
{
    uint8_t source = *instruction & 0b00000111;
    SubWithFlags(sys, sys->Registers[RA], *GetRegOrMem(sys, source));
}

//CPI #     11111110          ZSPCA   Compare immediate with A
void icpi(System* sys, uint8_t* instruction)
{
    SubWithFlags(sys, sys->Registers[RA], *(instruction + 1));
}

//RLC       00000111          C       Rotate A left
void irlc(System* sys, uint8_t* instruction)
{
    sys->Flags = sys->Registers[RA] & 0b10000000 ? sys->Flags | FC : sys->Flags & ~FC;
    sys->Registers[RA] = sys->Registers[RA] << 1 | sys->Registers[RA] >> 7;
}

//RRC       00001111          C       Rotate A right
void irrc(System* sys, uint8_t* instruction)
{
    sys->Flags = sys->Registers[RA] & 1 ? sys->Flags | FC : sys->Flags & ~FC;
    sys->Registers[RA] = sys->Registers[RA] >> 1 | sys->Registers[RA] << 7;
}

//RAL       00010111          C       Rotate A left through carry
void iral(System* sys, uint8_t* instruction)
{
    uint8_t carry = sys->Flags & FC;
    sys->Flags = sys->Registers[RA] & 0b10000000 ? sys->Flags | FC : sys->Flags & ~FC;
    sys->Registers[RA] = sys->Registers[RA] << 1 | sys->Registers[RA] >> 7;
    sys->Registers[RA] = carry ? sys->Registers[RA] | 1 : sys->Registers[RA] & ~1;
}

//RAR       00011111          C       Rotate A right through carry
void irar(System* sys, uint8_t* instruction)
{
    uint8_t carry = sys->Flags & FC;
    sys->Flags = sys->Registers[RA] & 1 ? sys->Flags | FC : sys->Flags & ~FC;
    sys->Registers[RA] = sys->Registers[RA] >> 1 | sys->Registers[RA] << 7;
    sys->Registers[RA] = carry ? sys->Registers[RA] | 0b10000000 : sys->Registers[RA] & 0b01111111;
}

//CMA       00101111          -       Compliment A
void icma(System* sys, uint8_t* instruction)
{
    sys->Registers[RA] = ~sys->Registers[RA];
}

//CMC       00111111          C       Complement Carry flag
void icmc(System* sys, uint8_t* instruction)
{
    sys->Flags ^= FC;
}

//STC       00110111          C       Set Carry flag
void istc(System* sys, uint8_t* instruction)
{
    sys->Flags |= FC;
}

//JMP a     11000011 lb hb    -       Unconditional jump
void ijmp(System* sys, uint8_t* instruction)
{
    sys->PC = *(uint16_t*)(instruction + 1);
}

//Jccc a    11CCC010 lb hb    -       Conditional jump
void ijccc(System* sys, uint8_t* instruction)
{
    uint8_t condition = (*instruction & 0b00111000) >> 3;
    condition = CheckCondition(sys, condition);

    if (condition)
        sys->PC = *(uint16_t*)(instruction + 1);
}

//CALL a    11001101 lb hb    -       Unconditional subroutine call
void icall(System* sys, uint8_t* instruction)
{
    PushOnStack(sys, sys->PC);
    sys->PC = *(uint16_t*)(instruction + 1);
}

//Cccc a    11CCC100 lb hb    -       Conditional subroutine call
void icccc(System* sys, uint8_t* instruction)
{
    uint8_t condition = (*instruction & 0b00111000) >> 3;
    condition = CheckCondition(sys, condition);

    if (condition)
    {
        PushOnStack(sys, sys->PC);
        sys->PC = *(uint16_t*)(instruction + 1);
    }
}

//RET       11001001          -       Unconditional return from subroutine
void iret(System* sys, uint8_t* instruction)
{
    sys->PC = PopFromStack(sys);
}

//Rccc      11CCC000          -       Conditional return from subroutine
void irccc(System* sys, uint8_t* instruction)
{
    uint8_t condition = (*instruction & 0b00111000) >> 3;
    condition = CheckCondition(sys, condition);

    if (condition)
        sys->PC = PopFromStack(sys);
}

//RST n     11NNN111          -       Restart (Call n*8)
void irst(System* sys, uint8_t* instruction)
{
    uint16_t address = *instruction & 0b00111000;
    PushOnStack(sys, sys->PC);
    sys->PC = address;
}

//PCHL      11101001          -       Jump to address in H:L
void ipchl(System* sys, uint8_t* instruction)
{
    sys->PC = GetPair(sys, PH);
}

//PUSH RP   11RP0101 *2       -       Push register pair on the stack
void ipush(System* sys, uint8_t* instruction)
{
    uint8_t pair = (*instruction & 0b00110000) >> 4;
    if (pair == PSP)
        pair = PSW;

    PushOnStack(sys, GetPair(sys, pair));
}

//POP RP    11RP0001 *2       *2      Pop  register pair from the stack
void ipop(System* sys, uint8_t* instruction)
{
    uint8_t pair = (*instruction & 0b00110000) >> 4;
    if (pair == PSP)
        pair = PSW;

    SetPair(sys, pair, PopFromStack(sys));
}

//XTHL      11100011          -       Swap H:L with top word on stack
void ixthl(System* sys, uint8_t* instruction)
{
    uint16_t value = GetPair(sys, PH);
    SetPair(sys, PH, PopFromStack(sys));
    PushOnStack(sys, value);
}

//SPHL      11111001          -       Set SP to content of H:L
void isphl(System* sys, uint8_t* instruction)
{
    sys->SP = GetPair(sys, PH);
}

//IN p      11011011 pa       -       Read input port into A
void iin(System* sys, uint8_t* instruction)
{
    uint8_t port = *(instruction + 1);
    if (sys->PeripheryCount == 0 || port > sys->PeripheryCount - 1 || sys->Periphery[port] == NULL)
        return;

    sys->Registers[RA] = (*(PeripheryPtr)sys->Periphery[port])(sys, 0, 1);
}

//OUT p     11010011 pa       -       Write A to output port
void iout(System* sys, uint8_t* instruction)
{
    uint8_t port = *(instruction + 1);
    if (sys->PeripheryCount == 0 || port > sys->PeripheryCount - 1 || sys->Periphery[port] == NULL)
        return;

    (*(PeripheryPtr)sys->Periphery[port])(sys, sys->Registers[RA], 0);
}

//EI        11111011          -       Enable interrupts
void iei(System* sys, uint8_t* instruction)
{
    sys->InterruptMode = INT_ENABLED;
}

//DI        11110011          -       Disable interrupts
void idi(System* sys, uint8_t* instruction)
{
    sys->InterruptMode = INT_DISABLED;
}

//HLT       01110110          -       Halt processor
void ihlt(System* sys, uint8_t* instruction)
{
    sys->Halt = 1;
}

//NOP       00000000          -       No operation
void inop(System* sys, uint8_t* instruction)
{
}

#define INSTRUCTION_COUNT 57

//mask, ambigious bits (for instruction with data encoded into them), size
const static uint8_t InstructionLookup[INSTRUCTION_COUNT * 3] =
{
    0b00000000, 0b00000000, 1, //nop
    0b01000000, 0b00111111, 1, //mov
    0b00000110, 0b00111000, 2, //mvi
    0b00000001, 0b00110000, 3, //lxi
    0b00111010, 0b00000000, 3, //lda
    0b00110010, 0b00000000, 3, //sta
    0b00101010, 0b00000000, 3, //lhld
    0b00100010, 0b00000000, 3, //shld
    0b00001010, 0b00110000, 1, //ldax
    0b00000010, 0b00110000, 1, //stax
    0b11101011, 0b00000000, 1, //xchg
    0b10000000, 0b00000111, 1, //add
    0b11000110, 0b00000000, 2, //adi
    0b10001000, 0b00000111, 1, //adc
    0b11001110, 0b00000000, 2, //aci
    0b10010000, 0b00000111, 1, //sub
    0b11010110, 0b00000000, 2, //sui
    0b10011000, 0b00000111, 1, //sbb
    0b11011110, 0b00000000, 2, //sbi
    0b00000100, 0b00111000, 1, //inr
    0b00000101, 0b00111000, 1, //dcr
    0b00000011, 0b00110000, 1, //inx
    0b00001011, 0b00110000, 1, //dcx
    0b00001001, 0b00110000, 1, //dad
    0b00100111, 0b00000000, 1, //daa
    0b10100000, 0b00000111, 1, //ana
    0b11100110, 0b00000000, 2, //ani
    0b10110000, 0b00000111, 1, //ora
    0b11110110, 0b00000000, 2, //ori
    0b10101000, 0b00000111, 1, //xra
    0b11101110, 0b00000000, 2, //xri
    0b10111000, 0b00000111, 1, //cmp
    0b11111110, 0b00000000, 2, //cpi
    0b00000111, 0b00000000, 1, //rlc
    0b00001111, 0b00000000, 1, //rrc
    0b00010111, 0b00000000, 1, //ral
    0b00011111, 0b00000000, 1, //rar
    0b00101111, 0b00000000, 1, //cma
    0b00111111, 0b00000000, 1, //cmc
    0b00110111, 0b00000000, 1, //stc
    0b11000011, 0b00000000, 3, //jmp
    0b11000010, 0b00111000, 3, //jccc
    0b11001101, 0b00000000, 3, //call
    0b11000100, 0b00111000, 3, //cccc
    0b11001001, 0b00000000, 1, //ret
    0b11000000, 0b00111000, 1, //rccc
    0b11000111, 0b00111000, 1, //rst
    0b11101001, 0b00000000, 1, //pchl
    0b11000101, 0b00110000, 1, //push
    0b11000001, 0b00110000, 1, //pop
    0b11100011, 0b00000000, 1, //xthl
    0b11111001, 0b00000000, 1, //sphl
    0b11011011, 0b00000000, 2, //in
    0b11010011, 0b00000000, 2, //out
    0b11111011, 0b00000000, 1, //ei
    0b11110011, 0b00000000, 1, //di
    0b01110110, 0b00000000, 1, //hlt
};

const static InstructionPtr InstructionPointers[INSTRUCTION_COUNT] =
{
    &inop,
    &imov,
    &imvi,
    &ilxi,
    &ilda,
    &ista,
    &ilhld,
    &ishld,
    &ildax,
    &istax,
    &ixchg,
    &iadd,
    &iadi,
    &iadc,
    &iaci,
    &isub,
    &isui,
    &isbb,
    &isbi,
    &iinr,
    &idcr,
    &iinx,
    &idcx,
    &idad,
    &idaa,
    &iana,
    &iani,
    &iora,
    &iori,
    &ixra,
    &ixri,
    &icmp,
    &icpi,
    &irlc,
    &irrc,
    &iral,
    &irar,
    &icma,
    &icmc,
    &istc,
    &ijmp,
    &ijccc,
    &icall,
    &icccc,
    &iret,
    &irccc,
    &irst,
    &ipchl,
    &ipush,
    &ipop,
    &ixthl,
    &isphl,
    &iin,
    &iout,
    &iei,
    &idi,
    &ihlt,
};

#pragma endregion Instructions

int ResolveInstruction(uint8_t opcode)
{
    for (int i = 0; i < INSTRUCTION_COUNT; i++)
    {
        uint8_t mask = InstructionLookup[i * 3];
        uint8_t ambMask = InstructionLookup[i * 3 + 1];

        mask ^= opcode;
        mask &= ~ambMask;
        if (mask)
            continue;

        return i;
    }

    return -1;
}

void Step(System* sys)
{
    sys->Flags |= FU;

    if (sys->InterruptMode == INT_RECEIVED)
    {
        sys->InterruptMode = INT_DISABLED;
        sys->Halt = 0;

        int index = ResolveInstruction(sys->Interrupt);
        if (index == -1)
            index = 0;

        (*InstructionPointers[index])(sys, sys->Memory + sys->PC);
    }

    if (sys->Halt)
        return;

    if (sys->Debug)
    {
        printf
        (
            "*PC: %04x SP: %04x SPW: %04x BC: %04x DE: %04x HL: %04x\nMEM: %02x %02x %02x %02x\nHALT: %i INT: %i\n",
            sys->PC,
            sys->SP,
            GetPair(sys, PSW),
            GetPair(sys, PB),
            GetPair(sys, PD),
            GetPair(sys, PH),
            *(sys->Memory + sys->PC),
            *(sys->Memory + sys->PC + 1),
            *(sys->Memory + sys->PC + 2),
            *(sys->Memory + sys->PC + 3),
            sys->Halt,
            sys->InterruptMode
        );
    }

    uint8_t* instruction = sys->Memory + sys->PC;
    int index = ResolveInstruction(*instruction);
    if (index == -1)
        index = 0;

    sys->PC += InstructionLookup[index * 3 + 2];
    (*InstructionPointers[index])(sys, instruction);
}

void Interrupt(System* sys, uint8_t instruction)
{
    if (sys->InterruptMode != INT_ENABLED)
        return;

    sys->Interrupt = instruction;
    sys->InterruptMode = INT_RECEIVED;
}
