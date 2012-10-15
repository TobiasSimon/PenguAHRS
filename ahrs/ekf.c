
/*
 * original code from CHRobotics
 * modification by Tobias Simon
 */


#include <stdio.h>
#include <math.h>
#include <string.h>

#include "matrix3x3.h"
#include "ekf.h"


ekf_config_t gConfig;
ahrs_state_t ekf_state;


void ekf_init(void)
{
   int i, j;
   memset(&ekf_state, 0, sizeof(ahrs_state_t));

   for (i = 0; i < 3; i++ )
   {
      ekf_state.P.data[i][i] = 0.01;
   }
 
   /* set process noise matrix: */
   ekf_state.Q.data[0][0] = gConfig.process_covariance;
   ekf_state.Q.data[1][1] = gConfig.process_covariance;
   ekf_state.Q.data[2][2] = gConfig.process_covariance;
 
   /* set measurement noise matrices: */
   ekf_state.R_acc.data[0][0] = gConfig.acc_covariance;
   ekf_state.R_acc.data[1][1] = gConfig.acc_covariance;
   ekf_state.R_acc.data[2][2] = gConfig.acc_covariance;
 
   ekf_state.R_mag.data[0][0] = gConfig.mag_covariance;
   ekf_state.R_mag.data[1][1] = gConfig.mag_covariance;
   ekf_state.R_mag.data[2][2] = gConfig.mag_covariance;
}


void ekf_sensor_init(raw_sensor_data_t *sensor_data)
{
   int i, j;
   memset(&ekf_state, 0, sizeof(ahrs_state_t));

   /* Compute initial roll and pitch angles */
   double theta_a = -atan2(sensor_data->acc.x, sensor_data->acc.z);
   double phi_a = -atan2(sensor_data->acc.y, sensor_data->acc.z);

   ekf_state.phi = phi_a;
   ekf_state.theta = theta_a;
   ekf_state.psi = 0;

   ekf_state.phi_dot = 0;
   ekf_state.theta_dot = 0;
   ekf_state.psi_dot = 0;

   ekf_state.P.data[0][0] = 0.01;
   ekf_state.P.data[1][1] = 0.01;
   ekf_state.P.data[1][1] = 0.01;

   for (i = 0; i < 3; i++ )
   {
      for (j = 0; j < 3; j++)
      {
         ekf_state.Q.data[i][j] = 0;
         ekf_state.R_acc.data[i][j] = 0;
         ekf_state.R_mag.data[i][j] = 0;
      }
   }

   /* set process noise matrix: */
   ekf_state.Q.data[0][0] = gConfig.process_covariance;
   ekf_state.Q.data[1][1] = gConfig.process_covariance;
   ekf_state.Q.data[2][2] = gConfig.process_covariance;

   /* set measurement noise matrices: */
   ekf_state.R_acc.data[0][0] = gConfig.acc_covariance;
   ekf_state.R_acc.data[1][1] = gConfig.acc_covariance;
   ekf_state.R_acc.data[2][2] = gConfig.acc_covariance;

   ekf_state.R_mag.data[0][0] = gConfig.mag_covariance;
   ekf_state.R_mag.data[1][1] = gConfig.mag_covariance;
   ekf_state.R_mag.data[2][2] = gConfig.mag_covariance;
}


static void unroll_states(ahrs_state_t *states)
{
   while (states->phi > M_PI)
   {
      states->phi -= M_PI;
   }

   while (states->phi < -M_PI)
   {
      states->phi += M_PI;
   }

   while (states->theta > M_PI)
   {
      states->theta -= M_PI;
   }

   while (states->theta < -M_PI)
   {
      states->theta += M_PI;
   }

   while (states->psi > M_PI)
   {
      states->psi -= M_PI;
   }

   while (states->psi < -M_PI)
   {
      states->psi += M_PI;
   }
}


static void ekf_predict(raw_sensor_data_t *sensor_data, double T)
{
   /* roll, pitch, yaw */
   double phi, theta, psi;
   double phi_dot, theta_dot, psi_dot;
   double p, q, r;

   mat3x3_t A, A_transpose, AP, PA_transpose, temp;
   int32_t i;

   /* scale gyro values: */
   vec3d_t pqr;
   vec_vec_elem_mul_3(&gConfig.gyro_scales, &sensor_data->gyro, &pqr);
   /* multiply gyro outputs by gyro alignment matrix: */
   mat_vect_mult3(&gConfig.gyro_alignment, &pqr, &pqr );
   p = pqr.x; q = pqr.y; r = pqr.z;

   /* get current state estimates: */
   phi = ekf_state.phi;
   theta = ekf_state.theta;
   psi = ekf_state.psi;
 
   double c_phi = cos(phi);
   double t_theta = tan(theta);
   double s_phi = sin(phi);
   double c_theta = cos(theta);
   double s_theta = sin(theta);
 
   /* compute expected angle rates based on gyro outputs 
      measured rotations  must be transformed into the inertial frame (we can't just integrate the gyro outputs): */
   phi_dot = p + s_phi * t_theta * q + c_phi * t_theta * r;
   theta_dot = c_phi * q - s_phi * r;
   psi_dot = (s_phi / c_theta) * q + (c_phi / c_theta) * r;
 
   ekf_state.phi_dot = phi_dot;
   ekf_state.theta_dot = theta_dot;
   ekf_state.psi_dot = psi_dot;

   /* compute new angle estimates: */
   phi = phi + T * phi_dot;
   theta = theta + T * theta_dot;
   psi = psi + T * psi_dot;
 
   /* linearize propogation equation for covariance estimation (build the matrix A): */
   #define A(i,j) A.data[i][j]
   A(0, 0) = q * c_phi * t_theta - r * s_phi * t_theta;   A(0, 1) = r * c_phi * (t_theta * t_theta + 1) + q * s_phi * (t_theta * t_theta + 1);               A(0, 2) = 0;
   A(1, 0) = -r * c_phi - q * s_phi;                      A(1, 1) = 0;                                                                                       A(1, 2) = 0;
   A(2, 0) = q * c_phi / c_theta - r * s_phi / c_theta;   A(2, 1) = r * c_phi * s_theta / (c_theta * c_theta) + q * s_phi * s_theta / (c_theta * c_theta);   A(2, 2) = 0;
 
   /* compute new covariance: P = P + T * (AP + PA ^ T + Q): */
   mat_mul_3x3(&A, &ekf_state.P, &AP );
   mat_trans_3x3(&A, &A_transpose);
   mat_mul_3x3(&ekf_state.P, &A_transpose, &PA_transpose);
   mat_add_3x3(&AP, &PA_transpose, &temp);
   mat_add_3x3(&temp, &ekf_state.Q, &temp);
   scalar_mat_mult_3x3(T, &temp, &temp);
   mat_add_3x3(&temp, &ekf_state.P, &ekf_state.P);

   /* update states estimates in data structure: */
   ekf_state.phi = phi;
   ekf_state.theta = theta;
   ekf_state.psi = psi;
   unroll_states(&ekf_state);
}


static void ekf_update(raw_sensor_data_t *sensor_data)
{
   mat3x3_t R, L, C, Ctrans, PCtrans, LC, temp, I;
   identity_3x3(&I);

   if (sensor_data->new_acc_data)
   {
      sensor_data->new_acc_data = 0;
      /* shortcut trigonometry definitions: */
      #define trig_short() \
         double theta = ekf_state.theta; double phi = ekf_state.phi; double psi = ekf_state.psi; \
         double c_phi = cos(phi); double c_theta = cos(theta); double c_psi = cos(psi); \
         double s_phi = sin(phi); double s_theta = sin(theta); double s_psi = sin(psi)
      trig_short();

      /* build rotation matrix from inertial frame to body frame
         for computing expected sensor outputs given yaw, pitch, and roll angles): */
      #define R(i,j) R.data[i][j]
      R(0, 0) = c_psi * c_theta;
      R(0, 1) = c_theta * s_psi;
      R(0, 2) = -s_theta;
      R(1, 0) = c_psi * s_phi * s_theta - c_phi * s_psi;
      R(1, 1) = c_phi * c_psi + s_phi * s_psi * s_theta;   
      R(1, 2) = c_theta * s_phi;
      R(2, 0) = s_phi * s_psi + c_phi * c_psi * s_theta;       
      R(2, 1) = c_phi * s_psi * s_theta - c_psi * s_phi;   
      R(2, 2) = c_phi * c_theta;

      /* subtract accel bias vector: */
      vec3d_t acc_ref;
      vec_sub_3(&gConfig.acc_ref, &gConfig.acc_biases, &acc_ref);

      /* multiply accel reference vector by calibration matrix: */
      mat_vect_mult3(&gConfig.acc_alignment, &acc_ref, &acc_ref);

      /* compute expected accelerometer output based on yaw, pitch, and roll angles: */
      vec3d_t acc_hat;
      mat_vect_mult3(&R, &acc_ref, &acc_hat);

      /* compute C matrix for Kalman gain calculation: */
      #define ax acc_ref.data[0]
      #define ay acc_ref.data[1]
      #define az acc_ref.data[2]
      #define C(i,j) C.data[i][j]
      C(0, 0) = 0;
      C(0, 1) = -az * c_theta - ax * c_psi * s_theta - ay * s_psi * s_theta;
      C(0, 2) = ay * c_psi * c_theta - ax * c_theta * s_psi;
      C(1, 0) = ax * (s_phi * s_psi + c_phi * c_psi * s_theta) - ay * (c_psi * s_phi - c_phi * s_psi * s_theta) + az * c_phi * c_theta;
      C(1, 1) = ax * c_psi * c_theta * s_phi - az * s_phi * s_theta + ay * c_theta * s_phi * s_psi;
      C(1, 2) = -ax * (c_phi * c_psi + s_phi * s_psi * s_theta) - ay * (c_phi * s_psi - c_psi * s_phi * s_theta);
      C(2, 0) = ax * (c_phi * s_psi - c_psi * s_phi * s_theta) - ay * (c_phi * c_psi + s_phi * s_psi * s_theta) - az * c_theta * s_phi;
      C(2, 1) = ax * c_phi * c_psi * c_theta - az * c_phi * s_theta + ay * c_phi * c_theta * s_psi;
      C(2, 2) = ax * (c_psi * s_phi - c_phi * s_psi * s_theta) + ay * (s_phi * s_psi + c_phi * c_psi * s_theta);

      /* compute kalman gain: L = PC^T * (R + CPC^T)^(-1) */
      mat_trans_3x3(&C, &Ctrans);
      mat_mul_3x3(&ekf_state.P, &Ctrans, &PCtrans);
      mat_mul_3x3(&C, &PCtrans, &temp);
      mat_add_3x3(&temp, &ekf_state.R_acc, &temp);
      mat_inv_3x3(&temp, &temp);
      mat_mul_3x3(&PCtrans, &temp, &L);

      /* compute new covariance: */
      mat_mul_3x3(&L, &C, &LC);
      scalar_mat_mult_3x3(-1, &LC, &temp);
      mat_add_3x3(&I, &temp, &temp);
      mat_mul_3x3(&temp, &ekf_state.P, &ekf_state.P);

      /* subtract accel bias from sensor readings: */
      vec3d_t acc_vec;
      vec_sub_3(&sensor_data->acc, &gConfig.acc_biases, &acc_vec);

      /* apply alignment correction */
      mat_vect_mult3(&gConfig.acc_alignment, &acc_vec, &acc_vec);
  
      /* subtract the reference vector: */
      vec_sub_3(&acc_vec, &acc_hat, &acc_vec);

      /* multiply by Kalman gain: */
      vec3d_t correction;
      mat_vect_mult3(&L, &acc_vec, &correction);

      /* only update pitch and roll: */
      ekf_state.phi = ekf_state.phi + correction.data[0];
      ekf_state.theta = ekf_state.theta + correction.data[1];

      /* "unroll" angle estimates to be in the range from -360 to 360 degrees */
      unroll_states(&ekf_state);
   }

   if (sensor_data->new_mag_data)
   {
      sensor_data->new_mag_data = 0;
      trig_short();
 
      /* get magnetic field reference vector and subtract bias vector: */
      vec3d_t mag_ref;
      vec_sub_3(&gConfig.mag_ref, &gConfig.mag_biases, &mag_ref);

      /* apply calibration matrix to mag reference vector: */
      mat_vect_mult3(&gConfig.mag_cal, &mag_ref, &mag_ref);

      /* compute C based on magnetic field data in inertial frame: */
      C(0, 0) = 0;   C(0, 1) = 0;   C(0, 2) = mag_ref.y * c_psi - mag_ref.x * s_psi;
      C(1, 0) = 0;   C(1, 1) = 0;   C(1, 2) = -mag_ref.x * c_psi - mag_ref.y * s_psi;
      C(2, 0) = 0;   C(2, 1) = 0;   C(2, 2) = 0;

      /* rotate reference vector into vehicle-1 frame: */
      R(0, 0) = c_psi;    R(0, 1) = s_psi;   R(0, 2) = 0;
      R(1, 0) = -s_psi;   R(1, 1) = c_psi;   R(1, 2) = 0;
      R(2, 0) = 0;        R(2, 1) = 0;       R(2, 2) = 1;
      mat_vect_mult3(&R, &mag_ref, &mag_ref);

      /* get magnetic field measurement and subtract bias: */
      vec3d_t mag_vect;
      vec_sub_3(&sensor_data->mag, &gConfig.mag_biases, &mag_vect);

      /* apply calibration matrix to magnetic field measurements: */
      mat_vect_mult3(&gConfig.mag_cal, &mag_vect, &mag_vect);

      /* Build rotation matrix from body frame to vehicle-1 frame (ie. only yaw remains uncorrected) */
      R(0, 0) = c_theta;    R(0, 1) = s_phi * s_theta;   R(0, 2) = c_phi * s_theta;
      R(1, 0) = 0;          R(1, 1) = c_phi;             R(1, 2) = -s_phi;
      R(2, 0) = -s_theta;   R(2, 1) = c_theta * s_phi;   R(2, 2) = c_phi * c_theta;
  
      /* rotate measurement into vehicle-1 frame: */
      mat_vect_mult3(&R, &mag_vect, &mag_vect);

      /* subtract reference vector: */
      vec_sub_3(&mag_vect, &mag_ref, &mag_vect);
  
      /* compute Kalman gain: L = PC^T * (R + CPC^T)^(-1): */
      mat_trans_3x3(&C, &Ctrans);
      mat_mul_3x3(&ekf_state.P, &Ctrans, &PCtrans);
      mat_mul_3x3(&C, &PCtrans, &temp);
      mat_add_3x3(&temp, &ekf_state.R_mag, &temp);
      mat_inv_3x3(&temp, &temp);
      mat_mul_3x3(&PCtrans, &temp, &L);
  
      /* Compute new covariance: */
      mat_mul_3x3(&L, &C, &LC);
      scalar_mat_mult_3x3(-1, &LC, &temp);
      mat_add_3x3(&I, &temp, &temp);
      mat_mul_3x3(&temp, &ekf_state.P, &ekf_state.P);
  
      /* Perform state update: */
      vec3d_t correction;
      mat_vect_mult3(&L, &mag_vect, &correction);
        
      /* only update yaw: */
      ekf_state.psi = ekf_state.psi + correction.data[2];
  
      /* "unroll" angle estimates to be in the range from -360 to 360 degrees: */
      unroll_states(&ekf_state);
   }
}


void ekf_run(raw_sensor_data_t *sensor_data, double T)
{
   ekf_predict(sensor_data, T);
   ekf_update(sensor_data);
}

