#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "system.h"
#include "misc.c"
#include "periphery/periphery.h"

int main(int argc, char** argv)
{
    System sys = {{0}};
    sys.Memory = malloc(MEMSIZE);
    memset(sys.Memory, 0, MEMSIZE);

    char* filePath = NULL;
    int stepInterval = 0;
    int offset = 0;

    for (int i = 0; i < argc; i++)
    {
        if (argv[i][0] == '-')
        {
            switch (argv[i][1])
            {
                case 'f':
                    filePath = argv[i + 1];
                    break;
                case 'i':
                    stepInterval = atoi(argv[i + 1]);
                    break;
                case 's':
                    sys.PC = (uint16_t) strtol(argv[i + 1], NULL, 16);
                    break;
                case 'o':
                    offset = strtol(argv[i + 1], NULL, 16);
                    break;
                case 'd':
                {
                    sys.Debug = (uint8_t) atoi(argv[i + 1]);
                }
            }
        }
    }

    if (filePath == NULL)
    {
        printf(
            "Arguments:\n"
            "-f path    - Program file (required).\n"
            "-i ms      - Step interval, 0 by default.\n"
            "-s address - Starting address, hex, 0 by default.\n"
            "-o address - File offset, hex, 0 by default.\n"
            "-d 0/1     - Debug output, 0 - disabled, 1 - enabled.\n"
        );
        return -1;
    }

    FILE* file = fopen(filePath, "rb");
    fread(sys.Memory + offset, sizeof(uint8_t), 0x10000, file);
    fclose(file);

    //cp/m bdos stub
    /*
    sys.Periphery = malloc(1);
    memset(sys.Periphery, 0, sizeof(PeripheryPtr));
    sys.PeripheryCount = 1;
    sys.Periphery[0] = &BdosCall;
    sys.Memory[0x0005] = 0xD3; //OUT
    sys.Memory[0x0006] = 0x00; //port 0
    sys.Memory[0x0007] = 0xC9; //RET
    */

    //simple console
    sys.Periphery = malloc(2);
    memset(sys.Periphery, 0, sizeof(PeripheryPtr) * 2);
    sys.PeripheryCount = 2;
    sys.Periphery[1] = &Console;

    while (1)
    {
        Step(&sys);
        if (stepInterval != 0)
            sleep(stepInterval);
    }

    return 0;
}
