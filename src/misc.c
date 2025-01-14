#ifdef _WIN32
#include <windows.h>
#endif

#ifdef __linux__
#include <unistd.h>
#include <time.h>
#include <errno.h>
#endif

void SleepMilli(int ms)
{
    #ifdef _WIN32
    Sleep(ms);
    #endif

    #ifdef __linux__
    //https://stackoverflow.com/questions/1157209/is-there-an-alternative-sleep-function-in-c-to-milliseconds
    struct timespec ts;
    int res;

    ts.tv_sec = ms / 1000;
    ts.tv_nsec = (ms % 1000) * 1000000;

    do {
        res = nanosleep(&ts, &ts);
    } while (res && errno == EINTR);
    #endif
}
