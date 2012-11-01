
/*
   interval computation interface

   Copyright (C) 2012 Tobias Simon

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
*/



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

