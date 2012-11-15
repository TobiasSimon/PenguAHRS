
/*
   PenguAHRS - A Linux-based Attitude and Heading Reference System

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


#include "kalman.h"
#include <stdio.h>


/* 
 * argv: "[process_var] [measure_var]"
 * stdin: "[acc] [raw] [dt]" 
 * stdout: "[acc] [raw] [filtered]"
 */
int main(int argc, void *argv[])
{
   kalman_t kalman;
   if (argc < 3)
   {
      return EXIT_FAILURE;
   }
   kalman_init(&kalman, atof(argv[1]), atof(argv[2]), 0.0f, 0.0f);
   char buffer[1024];
   int c, i = 0;
   while ((c = getchar()) != EOF)
   {
      if (c != '\n')
      {  
         buffer[i++] = c;
      }
      else
      {
         buffer[i] = '\0';
         kalman_in_t kalman_in;
         kalman_in.dt = 0.0033333;
         sscanf(buffer, "%f %f", &kalman_in.acc, &kalman_in.pos);
         kalman_out_t kalman_out;
         kalman_run(&kalman_out, &kalman, &kalman_in);
         printf("%f %f %f\n", kalman_in.pos, kalman_in.acc, kalman_out.pos);
         fflush(stdout);
         i = 0;
      }
   }
   return 0;
}

