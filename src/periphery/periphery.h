#include "../system.h"

#ifndef PERIPHERY_H
#define PERIPHERY_H

uint8_t Console(System* sys, uint8_t value, uint8_t isRead);

uint8_t BdosCall(System* sys, uint8_t value, uint8_t isRead);

#endif
