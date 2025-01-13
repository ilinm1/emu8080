#include "periphery.h"

uint8_t Console(System* sys, uint8_t value, uint8_t isRead)
{
    if (isRead)
        return (uint8_t)getchar();

    printf("%c", value);
    return 0;
}
