

#include "common_func.h"
#include "time.h"


uint64_t DRV_GetTime(void)
{
    struct timespec tp;
    clock_gettime(CLOCK_MONOTONIC, &tp);
    return tp.tv_sec * 1000000 + (uint64_t)tp.tv_nsec / 1000;
}
