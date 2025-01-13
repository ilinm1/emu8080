#include <stdint.h>
#include <stdio.h>

#ifndef SYSTEM_H
#define SYSTEM_H

//registers
#define RB 0
#define RC 1
#define RD 2
#define RE 3
#define RH 4
#define RL 5
#define RM 6
#define RA 7

//pairs
#define PB 0
#define PD 1
#define PH 2
#define PSP 3
#define PSW 4

//flags
#define FC 1                 //carry
#define FU (uint8_t)(1 << 1) //unused, always active
#define FP (uint8_t)(1 << 2) //parity
#define FA (uint8_t)(1 << 4) //aux carry
#define FZ (uint8_t)(1 << 6) //zero
#define FS (uint8_t)(1 << 7) //sign

//conditions
#define CNZ 0 //zero
#define CZ 1
#define CNC 2 //carry
#define CC 3
#define CNP 4 //parity
#define CP 5
#define CNS 6 //sign
#define CS 7

#define MEMSIZE 0x10000

#define INT_DISABLED 0
#define INT_ENABLED 1
#define INT_RECEIVED 2

typedef struct t_system
{
    uint8_t Registers[8];
    uint8_t Flags;
    uint8_t Halt;
    uint8_t Debug;

    uint16_t PC;
    uint16_t SP;

    uint8_t* Memory;

    uint8_t PeripheryCount;
    void** Periphery;

    uint8_t InterruptMode;
    uint8_t Interrupt;
} System;

typedef void (*InstructionPtr) (System* sys, uint8_t* instruction);
typedef uint8_t (*PeripheryPtr) (System* sys, uint8_t value, uint8_t isRead);

uint16_t GetPair(System* sys, uint8_t pair);
void SetPair(System* sys, uint8_t pair, uint16_t value);

void PushOnStack(System* sys, uint16_t n);
uint16_t PopFromStack(System* sys);

void Step(System* sys);
void Interrupt(System* sys, uint8_t instruction);
#endif
