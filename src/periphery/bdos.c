#include "periphery.h"

/*
for BDOS calls programs load function number into the C registry, parameter into DE pair, and call 0x05
so by injecting 'OUT port' at 0x05 and attaching this thing to that port it's possible to emulate BDOS functions without actual code
function list - https://www.seasip.info/Cpm/bdos.html, only basic IO functions are implemented but that's enough to run tests
*/
uint8_t BdosCall(System* sys, uint8_t value, uint8_t isRead)
{
    if (isRead)
        return 0;

    switch (sys->Registers[RC])
    {
        case 0: //reset
            sys->PC = 0;
            printf("BDOS RESET\n");
            break;
        case 1: //char input
            sys->Registers[RA] = sys->Registers[RL] = (uint8_t)getchar();
            break;
        case 2: //char output
            printf("%c", sys->Registers[RE]);
            break;
        case 9: //string output
            for (uint16_t i = GetPair(sys, PD); i < MEMSIZE - 1; i++)
            {
                uint8_t c = *(sys->Memory + i);
                if (c == '$')
                    break;

                printf("%c", c);
            }
            break;
        case 10: //string input
            uint16_t address = GetPair(sys, PD);
            uint8_t available = sys->Memory[address];
            gets_s(sys->Memory + address, available);
            break;
        case 11: //console status
            sys->Registers[RA] = sys->Registers[RL] = 0;
            break;
        case 12: //system version
            sys->Registers[RB] = sys->Registers[RH] = 0; //machine type - 8080
            sys->Registers[RA] = sys->Registers[RL] = 0; //cp/m version - cp/m 1
            break;
    }

    return 0;
}