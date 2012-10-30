
/*
   Math Utility Library

   Copyright (C) 2012 Tobias Simon

   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation; either version 2 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program; if not, write to the Free Software
   Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
   MA 02110-1301 USA.
*/


#ifndef __MATH_UTIL_H__
#define __MATH_UTIL_H__


typedef union
{
   struct
   {
      float x;
      float y;
      float z;
   };
   float vec[3];
}
vec3_t;


typedef union
{
   struct
   {
      float q0;
      float q1;
      float q2;
      float q3;
   };
   float vec[4];
}
quat_t;


typedef union
{
   struct
   {
      float yaw;
      float pitch;
      float roll;
   };
   float vec[3];
}
euler_t;


#endif /* __MATH_UTIL_H__ */

