
/*
 * sliding_avg.c - fast sliding average implementation
 *
 * Created on: 08.10.2011
 * Author: tobi
 */


#include <malloc.h>

#include "sliding_avg.h"


struct sliding_avg
{
   float *hist;
   int wnd_size;
   float sum;
   int pos;
};


sliding_avg_t *sliding_avg_create(int wnd_size, float init)
{
   sliding_avg_t *sliding_avg = malloc(sizeof(sliding_avg_t));
   sliding_avg->wnd_size = wnd_size + 1;
   sliding_avg->hist = malloc(sizeof(float) * sliding_avg->wnd_size);
   sliding_avg->sum = init * wnd_size;
   for (int i = 0; i < sliding_avg->wnd_size; i++)
   {
      sliding_avg->hist[i] = init;
   }
   sliding_avg->pos = 0;
   return sliding_avg;
}


float sliding_avg_calc(sliding_avg_t *sliding_avg, float val)
{
   int next_pos = (sliding_avg->pos + 1) % sliding_avg->wnd_size;
   float last = sliding_avg->hist[next_pos];
   sliding_avg->sum = sliding_avg->sum - last + val;
   sliding_avg->hist[sliding_avg->pos] = val;
   sliding_avg->pos = next_pos;
   return sliding_avg_get(sliding_avg);
}


float sliding_avg_get(sliding_avg_t *sliding_avg)
{
   return sliding_avg->sum / (sliding_avg->wnd_size - 1);
}


void sliding_avg_destroy(sliding_avg_t *sliding_avg)
{
   free(sliding_avg->hist);
   free(sliding_avg);
}

