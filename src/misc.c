#ifdef _WIN32
#include <windows.h>
#endif

void SleepMilli(int ms)
{
    #ifdef _WIN32
    Sleep(ms);
    #endif
}
