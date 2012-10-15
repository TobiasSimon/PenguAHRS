/* ------------------------------------------------------------------------------
  File: matrix3x3.c
  Author: CH Robotics
  Version: 1.0
  
  Description: Functions for performing operations on 3x3 matrices
------------------------------------------------------------------------------ */

#include "matrix3x3.h"
#include <math.h>
#include <stdio.h>

void mat_print_3x3(mat3x3_t *src)
{
   int i, j;
   for (i = 0; i < 3; i++)
   {
      for (j = 0; j < 3; j++ )
      {
         printf("%f\t", src->data[i][j]);
      }
      printf("\n");
   }
}


int vec_sub_3(vec3d_t *src1, vec3d_t *src2, vec3d_t *dest)
{
   int i;
   for (i = 0; i < 3; i++)
   {
      dest->data[i] = src1->data[i] - src2->data[i];
   }
   return 1;
}


/* element-wise vector multiplication: */
int vec_vec_elem_mul_3(vec3d_t *src1, vec3d_t *src2, vec3d_t *dest)
{
   int i;
   for (i = 0; i < 3; i++)
   {
      dest->data[i] = src1->data[i] * src2->data[i];
   }
   return 1;
}


/*******************************************************************************
* Function Name  : mat_add_3x3
* Input          : mat3x3_t* src1, mat3x3_t* src2
* Output         : mat3x3_t* dest
* Return         : 0 if failed, 1 if success
* Description    : 

Performs the operation: dest = src1 + src2
'dest' can be the same as either src1 or src2
*******************************************************************************/
int mat_add_3x3( mat3x3_t* src1, mat3x3_t* src2, mat3x3_t* dest )
{
	 int i, j;
	 
	 /*
	 sum.data[0][0] = src1->data[0][0] + src2->data[0][0];
	 sum.data[0][1] = src1->data[0][1] + src2->data[0][1];
	 sum.data[0][2] = src1->data[0][2] + src2->data[0][2];
	 
	 sum.data[1][0] = src1->data[1][0] + src2->data[1][0];
	 sum.data[1][1] = src1->data[1][1] + src2->data[1][1];
	 sum.data[1][2] = src1->data[1][2] + src2->data[1][2];
	 
	 sum.data[2][0] = src1->data[2][0] + src2->data[2][0];
	 sum.data[2][1] = src1->data[2][1] + src2->data[2][1];
	 sum.data[2][2] = src1->data[2][2] + src2->data[2][2];
	 */
	 
	 for (i = 0; i < 3; i++)
	 {
		  for (j = 0; j < 3; j++ )
		  {
				dest->data[i][j] = src1->data[i][j] + src2->data[i][j];
		  }
	 }
	 
	 return 1;
}

/*******************************************************************************
* Function Name  : mat_mul_3x3
* Input          : mat3x3_t* src1, mat3x3_t* src2
* Output         : mat3x3_t* dest
* Return         : 0 if failed, 1 if success
* Description    : 

Performs the operation: dest = src1*src2;

'dest' can be the same as either src1 or src2
*******************************************************************************/
int mat_mul_3x3( mat3x3_t* src1, mat3x3_t* src2, mat3x3_t* dest )
{
	 int i, j;
	 mat3x3_t result;
	 
	 // First row
	 result.data[0][0] = src1->data[0][0]*src2->data[0][0] + src1->data[0][1]*src2->data[1][0] + src1->data[0][2]*src2->data[2][0];
	 result.data[0][1] = src1->data[0][0]*src2->data[0][1] + src1->data[0][1]*src2->data[1][1] + src1->data[0][2]*src2->data[2][1];
	 result.data[0][2] = src1->data[0][0]*src2->data[0][2] + src1->data[0][1]*src2->data[1][2] + src1->data[0][2]*src2->data[2][2];
	 
	 // Second row
	 result.data[1][0] = src1->data[1][0]*src2->data[0][0] + src1->data[1][1]*src2->data[1][0] + src1->data[1][2]*src2->data[2][0];
	 result.data[1][1] = src1->data[1][0]*src2->data[0][1] + src1->data[1][1]*src2->data[1][1] + src1->data[1][2]*src2->data[2][1];
	 result.data[1][2] = src1->data[1][0]*src2->data[0][2] + src1->data[1][1]*src2->data[1][2] + src1->data[1][2]*src2->data[2][2];
	 
	 // Third row
	 result.data[2][0] = src1->data[2][0]*src2->data[0][0] + src1->data[2][1]*src2->data[1][0] + src1->data[2][2]*src2->data[2][0];
	 result.data[2][1] = src1->data[2][0]*src2->data[0][1] + src1->data[2][1]*src2->data[1][1] + src1->data[2][2]*src2->data[2][1];
	 result.data[2][2] = src1->data[2][0]*src2->data[0][2] + src1->data[2][1]*src2->data[1][2] + src1->data[2][2]*src2->data[2][2];
	 
	 for (i = 0; i < 3; i++)
	 {
		  for (j = 0; j < 3; j++ )
		  {
				dest->data[i][j] = result.data[i][j];
		  }
	 }
	 
	 return 1;
}

/*******************************************************************************
* Function Name  : mat_inv_3x3
* Input          : mat3x3_t* src1, mat3x3_t* src2
* Output         : mat3x3_t* dest
* Return         : 0 if failed, 1 if success
* Description    : 

Computes the inverse of the 3x3 matrix in src
*******************************************************************************/
int mat_inv_3x3( mat3x3_t* src, mat3x3_t* dest )
{
	 mat3x3_t inverse;
	 int i, j;
	 
	 double det = mat_det_3x3( src );
	 
	 // Invert the matrix
	 /*
	 | a11 a12 a13 |-1             |   a33a22-a32a23  -(a33a12-a32a13)   a23a12-a22a13  |
	 | a21 a22 a23 |    =  1/DET * | -(a33a21-a31a23)   a33a11-a31a13  -(a23a11-a21a13) |
	 | a31 a32 a33 |               |   a32a21-a31a22  -(a32a11-a31a12)   a22a11-a21a12  |

	 | a00 a01 a02 |-1             |   a22a11-a21a12  -(a22a01-a21a02)   a12a01-a11a02  |
	 | a10 a11 a12 |    =  1/DET * | -(a22a10-a20a12)   a22a00-a20a02  -(a12a00-a10a02) |
	 | a20 a21 a22 |               |   a21a10-a20a11  -(a21a00-a20a01)   a11a00-a10a01  |
	 */
	 
	 // Row 1
//	 inverse.data[0][0] = (a22a11-a21a12)/det;
	 inverse.data[0][0] = (src->data[2][2]*src->data[1][1]-src->data[2][1]*src->data[1][2])/det;
//	 inverse.data[0][1] = -(a22a01-a21a02)/det;
	 inverse.data[0][1] = -(src->data[2][2]*src->data[0][1]-src->data[2][1]*src->data[0][2])/det;
//	 inverse.data[0][2] = (a12a01-a11a02)/det;
	 inverse.data[0][2] = (src->data[1][2]*src->data[0][1]-src->data[1][1]*src->data[0][2])/det;
	 
	 // Row 2
//	 inverse.data[1][0] = -(a22a10-a20a12)/det;
	 inverse.data[1][0] = -(src->data[2][2]*src->data[1][0]-src->data[2][0]*src->data[1][2])/det;
//	 inverse.data[1][1] = (a22a00-a20a02)/det;
	 inverse.data[1][1] = (src->data[2][2]*src->data[0][0]-src->data[2][0]*src->data[0][2])/det;
//	 inverse.data[1][2] = -(a12a00-a10a02)/det;
	 inverse.data[1][2] = -(src->data[1][2]*src->data[0][0]-src->data[1][0]*src->data[0][2])/det;
	 
	 // Row 3
//	 inverse.data[2][0] = (a21a10-a20a11)/det;
	 inverse.data[2][0] = (src->data[2][1]*src->data[1][0]-src->data[2][0]*src->data[1][1])/det;
//	 inverse.data[2][1] = -(a21a00-a20a01)/det;
	 inverse.data[2][1] = -(src->data[2][1]*src->data[0][0]-src->data[2][0]*src->data[0][1])/det;
//	 inverse.data[2][2] = (a11a00-a10a01)/det;
	 inverse.data[2][2] = (src->data[1][1]*src->data[0][0]-src->data[1][0]*src->data[0][1])/det;
	 
	 for (i = 0; i < 3; i++)
	 {
		  for (j = 0; j < 3; j++ )
		  {
				dest->data[i][j] = inverse.data[i][j];
		  }
	 }
	 
	 return 1;
}

/*******************************************************************************
* Function Name  : mat_vect_mult3
* Input          : mat3x3_t* matrix, vec3d_t* vector
* Output         : mat3x3_t* dest
* Return         : 0 if failed, 1 if success
* Description    : 

Performs the operation: dest = matrix*vector
*******************************************************************************/
int mat_vect_mult3( mat3x3_t* matrix, vec3d_t* vector, vec3d_t* dest )
{
	 vec3d_t result;
	 int i;
	 
	 result.data[0] = matrix->data[0][0]*vector->data[0] + matrix->data[0][1]*vector->data[1] + matrix->data[0][2]*vector->data[2];
	 result.data[1] = matrix->data[1][0]*vector->data[0] + matrix->data[1][1]*vector->data[1] + matrix->data[1][2]*vector->data[2];
	 result.data[2] = matrix->data[2][0]*vector->data[0] + matrix->data[2][1]*vector->data[1] + matrix->data[2][2]*vector->data[2];
	 
	 for (i = 0; i < 3; i++)
	 {
		  dest->data[i] = result.data[i];
	 }
	 
	 return 1;
}

/*******************************************************************************
* Function Name  : mat_det_3x3
* Input          : mat3x3_t* src
* Output         : None
* Return         : The determinant of the matrix
* Description    : 

Computes the determinant of the matrix in 'src'
*******************************************************************************/
double mat_det_3x3( mat3x3_t* src )
{
	 // det = a11(a33a22-a32a23)-a21(a33a12-a32a13)+a31(a23a12-a22a13)
	 // det = a00(a22a11-a21a12)-a10(a22a01-a21a02)+a20(a12a01-a11a02)
	 	 
	 return src->data[0][0]*(src->data[2][2]*src->data[1][1]-src->data[2][1]*src->data[1][2])
									 - src->data[1][0]*(src->data[2][2]*src->data[0][1]-src->data[2][1]*src->data[0][2])
									 + src->data[2][0]*(src->data[1][2]*src->data[0][1]-src->data[1][1]*src->data[0][2]);
}

/*******************************************************************************
* Function Name  : mat_trans_3x3
* Input          : mat3x3_t* src 
* Output         : mat3x3_t* dest
* Return         : 0 if failed, 1 if success
* Description    : 

Transposes the matrix in 'src', stores the result in 'dest'
*******************************************************************************/
int mat_trans_3x3( mat3x3_t* src, mat3x3_t* dest )
{
	 mat3x3_t temp;
	 int i,j;
	 
	 temp.data[0][0] = src->data[0][0];
	 temp.data[0][1] = src->data[1][0];
	 temp.data[0][2] = src->data[2][0];
	 
	 temp.data[1][0] = src->data[0][1];
	 temp.data[1][1] = src->data[1][1];
	 temp.data[1][2] = src->data[2][1];
	 
	 temp.data[2][0] = src->data[0][2];
	 temp.data[2][1] = src->data[1][2];
	 temp.data[2][2] = src->data[2][2];
	 
	 for (i = 0; i < 3; i++)
	 {
		  for (j = 0; j < 3; j++ )
		  {
				dest->data[i][j] = temp.data[i][j];
		  }
	 }
	 
	 return 1;	 
}

/*******************************************************************************
* Function Name  : scalar_mat_mult_3x3
* Input          : double scal, mat3x3_t* src 
* Output         : mat3x3_t* dest
* Return         : 0 if failed, 1 if success
* Description    : 

Multiplies the elements in src by scal and stores the result in dest

*******************************************************************************/
int scalar_mat_mult_3x3( double scal, mat3x3_t* src, mat3x3_t* dest )
{
	 dest->data[0][0] = src->data[0][0]*scal;
	 dest->data[0][1] = src->data[0][1]*scal;
	 dest->data[0][2] = src->data[0][2]*scal;
	 
	 dest->data[1][0] = src->data[1][0]*scal;
	 dest->data[1][1] = src->data[1][1]*scal;
	 dest->data[1][2] = src->data[1][2]*scal;
	 
	 dest->data[2][0] = src->data[2][0]*scal;
	 dest->data[2][1] = src->data[2][1]*scal;
	 dest->data[2][2] = src->data[2][2]*scal;
	 
	 return 1;
}

/*******************************************************************************
* Function Name  : identity_3x3
* Input          : None
* Output         : mat3x3_t* dest
* Return         : Void
* Description    : 

Creates a 3x3 identity matrix

*******************************************************************************/
void identity_3x3( mat3x3_t* dest )
{
	 mat_zero_3x3( dest );
	 
	 dest->data[0][0] = 1;
	 dest->data[1][1] = 1;
	 dest->data[2][2] = 1;
}

/*******************************************************************************
* Function Name  : mat_zero_3x3
* Input          : None
* Output         : mat3x3_t* dest
* Return         : Void
* Description    : 

Zeros out all the entries in dest

*******************************************************************************/
void mat_zero_3x3( mat3x3_t* dest )
{
	 dest->data[0][0] = 0;
	 dest->data[0][1] = 0;
	 dest->data[0][2] = 0;
	 
	 dest->data[1][0] = 0;
	 dest->data[1][1] = 0;
	 dest->data[1][2] = 0;
	 
	 dest->data[2][0] = 0;
	 dest->data[2][1] = 0;
	 dest->data[2][2] = 0;
}

/*******************************************************************************
* Function Name  : mat_copy_3x3
* Input          : mat3x3_t* src
* Output         : mat3x3_t* dest
* Return         : Void
* Description    : 

Copies the data in src to dest.

*******************************************************************************/
void mat_copy_3x3( mat3x3_t* src, mat3x3_t* dest )
{
	 dest->data[0][0] = src->data[0][0];
	 dest->data[0][1] = src->data[0][1];
	 dest->data[0][2] = src->data[0][2];
	 
	 dest->data[1][0] = src->data[1][0];
	 dest->data[1][1] = src->data[1][1];
	 dest->data[1][2] = src->data[1][2];
	 
	 dest->data[2][0] = src->data[2][0];
	 dest->data[2][1] = src->data[2][1];
	 dest->data[2][2] = src->data[2][2];
}
