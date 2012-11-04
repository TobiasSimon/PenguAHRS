
/*
 * sliding_avg.h - fast sliding average interface
 *
 * Created on: 08.10.2011
 * Author: tobi
 */

#ifndef __SLIDING_AVG_H__
#define __SLIDING_AVG_H__


struct sliding_avg;
typedef struct sliding_avg sliding_avg_t;


sliding_avg_t *sliding_avg_create(int wnd_size, float init);

float sliding_avg_calc(sliding_avg_t *sliding_avg, float val);

float sliding_avg_get(sliding_avg_t *sliding_avg);

void sliding_avg_destroy(sliding_avg_t *sliding_avg);


#endif /* __SLIDING_AVG_H__ */

