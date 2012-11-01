
/*
   mag_decl.h magnetic declination lookup table interface

   Copyright (C) 2012 Tobias Simon

   Copyright (C) Adam M Rivera
   With direction from: Andrew Tridgell, Jason Short, Justin Beech
   Adapted from: http://www.societyofrobots.com/robotforum/index.php?topic=11855.0
   Scott Ferguson
   scottfromscott@gmail.com
 
   This program is free software; you can redistribute it and/or modify
   it under the terms of the GNU Lesser General Public License as published
   by the Free Software Foundation; either version 2.1 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU Lesser General Public License for more details.
*/


#ifndef __MAG_DECL_H__
#define __MAG_DECL_H__

/* returns magnetic declination in degrees
   for a given latitude/longitude in degrees */
float get_declination(float lat, float lon);


#endif // __MAG_DECL_H__
