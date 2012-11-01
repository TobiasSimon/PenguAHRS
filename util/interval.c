

#include <stdint.h>

#include "interval.h"


void interval_init(interval_t *interval)
{
   clock_gettime(CLOCK_MONOTONIC, &interval->prev);
}


static int64_t ts_diff(struct timespec *timeA_p, struct timespec *timeB_p)
{
   return ((timeA_p->tv_sec * 1000000000) + timeA_p->tv_nsec) -
          ((timeB_p->tv_sec * 1000000000) + timeB_p->tv_nsec);
}


float interval_measure(interval_t *interval)
{
   clock_gettime(CLOCK_MONOTONIC, &interval->curr);
   float dt = (float)ts_diff(&interval->curr, &interval->prev) / 1000000000.0;
   interval->prev = interval->curr;
   return dt;
}


