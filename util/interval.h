

#ifndef __INTERVAL_H__
#define __INTERVAL_H__


#include <time.h>


typedef struct
{
   struct timespec prev;
   struct timespec curr;
}
interval_t;


void interval_init(interval_t *interval);

float interval_measure(interval_t *interval);


#endif /* __INTERVAL_H__ */

