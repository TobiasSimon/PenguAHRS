
#include <math.h>

#include "../util/math.h"


void quaternion_init(quat_t *quat, float ax, float ay, float az, float mx, float my, float mz)
{
   float init_roll = atan2(-ay, -az);
   float init_pitch = atan2(ax, -az);

   float cos_roll = cosf(init_roll);
   float sin_roll = sinf(init_roll);
   float cos_pitch = cosf(init_pitch);
   float sin_pitch = sinf(init_pitch);

   float mag_x = mx * cos_pitch + my * sin_roll * sin_pitch + mz * cos_roll * sin_pitch;
   float mag_y = my * cos_roll - mz * sin_roll;

   float init_yaw = atan2(-mag_y, mag_x);

   cos_roll =  cosf(init_roll * 0.5f);
   sin_roll =  sinf(init_roll * 0.5f);

   cos_pitch = cosf(init_pitch * 0.5f );
   sin_pitch = sinf(init_pitch * 0.5f );

   float cosHeading = cosf(init_yaw * 0.5f);
   float sinHeading = sinf(init_yaw * 0.5f);

   quat->q0 = cos_roll * cos_pitch * cosHeading + sin_roll * sin_pitch * sinHeading;
   quat->q1 = sin_roll * cos_pitch * cosHeading - cos_roll * sin_pitch * sinHeading;
   quat->q2 = cos_roll * sin_pitch * cosHeading + sin_roll * cos_pitch * sinHeading;
   quat->q3 = cos_roll * cos_pitch * sinHeading - sin_roll * sin_pitch * cosHeading;
}



void quat_to_euler(euler_t *euler, quat_t *quat)
{
   float s = quat->q0;
   struct
   {
      float x;
      float y;
      float z;
   }
   v;
   v.x = quat->q1;
   v.y = quat->q2;
   v.z = quat->q3;  

   float sqw = s*s;    
   float sqx = v.x*v.x;    
   float sqy = v.y*v.y;    
   float sqz = v.z*v.z;    

   euler->yaw = atan2f(2.f * (v.x*v.y + v.z*s), sqx - sqy - sqz + sqw);          
   euler->pitch = asinf(-2.f * (v.x*v.z - v.y*s));
   euler->roll = atan2f(2.f * (v.y*v.z + v.x*s), -sqx - sqy + sqz + sqw);    
}


float inv_sqrt(float x)
{
   float halfx = 0.5f * x;
   float y = x;
   long i = *(long*)&y;
   i = 0x5f3759df - (i>>1);
   y = *(float*)&i;
   y = y * (1.5f - (halfx * y * y));
   return y;
}

