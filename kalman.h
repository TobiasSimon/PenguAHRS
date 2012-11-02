
/*
 * file: kalman.h
 * 
 * implements kalman filter for linear system:
 *
 * | 1 dt | * | p | + | 0.5 * dt ^ 2 | * | a | = | p |
 * | 0  1 | * | v |   |           dt |           | v |
 * 
 * authors:
 *    Jan Roemisch, Ilmenau University of Technology
 *    Tobias Simon, Ilmenau University of Technology
 */


#ifndef __KALMAN_H__
#define __KALMAN_H__


#include <meschach/matrix.h>



typedef struct
{
   /* configuration and constant matrices: */
   MAT *Q; /* process noise */
   MAT *R; /* measurement noise */
   MAT *I; /* identity matrix */

   /* state and transition vectors/matrices: */
   VEC *x; /* state (location and velocity) */
   VEC *z; /* measurement (location) */
   MAT *A; /* system matrix */
   MAT *B; /* control matrix */
   MAT *P; /* error covariance */
   VEC *u; /* control (acceleration) */
   MAT *H; /* observer matrix */
   MAT *K; /* kalman gain */

   /*  vectors and matrices for calculations: */
   VEC *t0;
   VEC *t1;
   MAT *T0;
   MAT *T1;
}
kalman_t;


typedef struct
{
   float pos;
   float speed;
}
kalman_out_t;


typedef struct
{
   float dt; /* time elapsed since last kalman step */
   float pos; /* position in m */
   float acc; /* acceleration min m/s^2 */
}
kalman_in_t;


/*
 * executes kalman predict and correct step
 */
void kalman_run(kalman_out_t *out, kalman_t *kalman, const kalman_in_t *in);


/*
 * initializes a kalman filter
 */
void kalman_init(kalman_t *kf, float q, float r, float pos, float speed);


#endif /* __KALMAN_H__ */

