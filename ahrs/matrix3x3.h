/* ------------------------------------------------------------------------------
  File: matrix3x3.h
  Author: CH Robotics
  Version: 1.0
  
  Description: Functions for performing operations on 3x3 matrices
------------------------------------------------------------------------------ */

#ifndef __MATRIX3X3_H_
#define __MATRIX3X3_H_

// Structure for holding 3x3 matrices
typedef struct _mat3x3_t {
	 double data[3][3];
} mat3x3_t;

typedef union
{
   struct
   {
      double x;
      double y;
      double z;
   };
   double data[3];
}
vec3d_t;


void mat_print_3x3(mat3x3_t *src);

int vec_sub_3(vec3d_t *src1, vec3d_t *src2, vec3d_t *dest);

int vec_vec_elem_mul_3(vec3d_t *src1, vec3d_t *src2, vec3d_t *dest);

int mat_add_3x3( mat3x3_t* src1, mat3x3_t* src2, mat3x3_t* dest );

int mat_mul_3x3( mat3x3_t* src1, mat3x3_t* src2, mat3x3_t* dest );

int mat_inv_3x3( mat3x3_t* src, mat3x3_t* dest );

int mat_vect_mult3( mat3x3_t* matrix, vec3d_t* vector, vec3d_t* dest );

double mat_det_3x3( mat3x3_t* src );

int mat_trans_3x3( mat3x3_t* src, mat3x3_t* dest );

int scalar_mat_mult_3x3( double T, mat3x3_t* src, mat3x3_t* dest );

void identity_3x3( mat3x3_t* dest );

void mat_zero_3x3( mat3x3_t* dest );

void mat_copy_3x3( mat3x3_t* src, mat3x3_t* dest );

#endif
