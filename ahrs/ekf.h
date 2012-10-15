
/*
 * original code from CHRobotics
 * modification by Tobias Simon
 */


#ifndef __EKF_H__
#define __EKF_H__


#include <stdint.h>
#include "matrix3x3.h"


typedef struct 
{
   /* gyro scaling: */
   vec3d_t gyro_scales;

   /* biases: */
   vec3d_t acc_biases;
   vec3d_t gyro_biases;
   vec3d_t mag_biases;

   /* covariances: */
   double process_covariance;
   double acc_covariance;
   double mag_covariance;

   /* alignments/calibration: */
   mat3x3_t gyro_alignment;
   mat3x3_t acc_alignment;
   mat3x3_t mag_cal;

   /*reference vectors: */
   vec3d_t mag_ref;
   vec3d_t acc_ref;
}
ekf_config_t;


typedef struct
{
   vec3d_t gyro;

   int new_acc_data;
   vec3d_t acc;

   int new_mag_data;
   vec3d_t mag;
}
raw_sensor_data_t;


typedef struct
{
   /* prientation states */
   union 
   {
      double heading;
      double yaw;
      double psi;
   };

   union 
   {
      double pitch;
      double theta;
   };

   union 
   {
      double roll;
      double phi;
   };


   /* orientation rate states */
   union 
   {
      double heading_rate;
      double yaw_rate;
      double psi_dot;
   };


   union 
   {
      double pitch_rate;
      double theta_dot;
   };

   union 
   {
      double roll_rate;
      double phi_dot;
   };

   /* process noise matrix: */
   mat3x3_t Q;
 
   /* measurement noise matrices: */
   mat3x3_t R_acc;
   mat3x3_t R_mag;
 
   /* EKF covariance */
   mat3x3_t P;
}
ahrs_state_t;


extern ekf_config_t gConfig;
extern ahrs_state_t ekf_state;

void ekf_init(void);
void ekf_sensor_init(raw_sensor_data_t *sensor_data);
void ekf_run(raw_sensor_data_t *sensor_data, double T);


#endif
