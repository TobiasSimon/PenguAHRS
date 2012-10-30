//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================


#include <math.h>
#include <stdio.h>
#include "util.h"
#include "madgwick_ahrs.h"


void madgwick_ahrs_init(madgwick_ahrs_t *ahrs, float beta)
{
   ahrs->quat.q0 = 1;
   ahrs->quat.q1 = 0;
   ahrs->quat.q2 = 0;
   ahrs->quat.q3 = 0;
   ahrs->beta = beta;
}


static void madgwick_ahrs_update_imu(madgwick_ahrs_t *ahrs,
                           float gx, float gy, float gz,
                           float ax, float ay, float az,
                           float accelCutoff, float dt)
{
	float accelSquareSum, recipNorm;
	float s0, s1, s2, s3;
	float qDot0, qDot1, qDot2, qDot3;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot0 = 0.5f * (-ahrs->quat.q1 * gx - ahrs->quat.q2 * gy - ahrs->quat.q3 * gz);
	qDot1 = 0.5f * ( ahrs->quat.q0 * gx + ahrs->quat.q2 * gz - ahrs->quat.q3 * gy);
	qDot2 = 0.5f * ( ahrs->quat.q0 * gy - ahrs->quat.q1 * gz + ahrs->quat.q3 * gx);
	qDot3 = 0.5f * ( ahrs->quat.q0 * gz + ahrs->quat.q1 * gy - ahrs->quat.q2 * gx);

	accelSquareSum = ax * ax + ay * ay + az * az;

	// Compute feedback only if accelerometer abs(vector) less than cutoff value (also avoids NaN in accelerometer normalisation)
	if (fabs(sqrt(accelSquareSum) - 9.8065) < accelCutoff)
	{
		// Normalise accelerometer measurement
		recipNorm = inv_sqrt(accelSquareSum);

		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * ahrs->quat.q0;
		_2q1 = 2.0f * ahrs->quat.q1;
		_2q2 = 2.0f * ahrs->quat.q2;
		_2q3 = 2.0f * ahrs->quat.q3;
		_4q0 = 4.0f * ahrs->quat.q0;
		_4q1 = 4.0f * ahrs->quat.q1;
		_4q2 = 4.0f * ahrs->quat.q2;
		_8q1 = 8.0f * ahrs->quat.q1;
		_8q2 = 8.0f * ahrs->quat.q2;
		q0q0 = ahrs->quat.q0 * ahrs->quat.q0;
		q1q1 = ahrs->quat.q1 * ahrs->quat.q1;
		q2q2 = ahrs->quat.q2 * ahrs->quat.q2;
		q3q3 = ahrs->quat.q3 * ahrs->quat.q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 - _2q2 * ax + _4q0 * q1q1 + _2q1 * ay;
		s1 = _4q1 * q3q3 + _2q3 * ax + 4.0f * q0q0 * ahrs->quat.q1 + _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 - _4q1 * az;
		s2 = 4.0f * q0q0 * ahrs->quat.q2 - _2q0 * ax + _4q2 * q3q3 + _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 - _4q2 * az;
		s3 = 4.0f * q1q1 * ahrs->quat.q3 + _2q1 * ax + 4.0f * q2q2 * ahrs->quat.q3 + _2q2 * ay;

		recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;
	   
      // Apply feedback step
	   qDot0 -= ahrs->beta * s0;
	   qDot1 -= ahrs->beta * s1;
	   qDot2 -= ahrs->beta * s2;
	   qDot3 -= ahrs->beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	ahrs->quat.q0 += qDot0 * dt;
	ahrs->quat.q1 += qDot1 * dt;
	ahrs->quat.q2 += qDot2 * dt;
	ahrs->quat.q3 += qDot3 * dt;

	// Normalise quaternion
	recipNorm = inv_sqrt(ahrs->quat.q0 * ahrs->quat.q0 + ahrs->quat.q1 * ahrs->quat.q1 + ahrs->quat.q2 * ahrs->quat.q2 + ahrs->quat.q3 * ahrs->quat.q3);
	ahrs->quat.q0 *= recipNorm;
	ahrs->quat.q1 *= recipNorm;
	ahrs->quat.q2 *= recipNorm;
	ahrs->quat.q3 *= recipNorm;
}



void madgwick_ahrs_update(madgwick_ahrs_t *ahrs, float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz,
                        float accelCutoff, float dt)
{
	float accelSquareSum, recipNorm;
	float s0, s1, s2, s3;
	float qDot0, qDot1, qDot2, qDot3;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz;
	float _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation) or
	// mag data has not been updated
	if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f))
	{
		madgwick_ahrs_update_imu(ahrs, gx, gy, gz,
		                      ax, ay, az,
		                      accelCutoff,
		                      dt);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot0 = 0.5f * (-ahrs->quat.q1 * gx - ahrs->quat.q2 * gy - ahrs->quat.q3 * gz);
	qDot1 = 0.5f * ( ahrs->quat.q0 * gx + ahrs->quat.q2 * gz - ahrs->quat.q3 * gy);
	qDot2 = 0.5f * ( ahrs->quat.q0 * gy - ahrs->quat.q1 * gz + ahrs->quat.q3 * gx);
	qDot3 = 0.5f * ( ahrs->quat.q0 * gz + ahrs->quat.q1 * gy - ahrs->quat.q2 * gx);

	accelSquareSum = ax * ax + ay * ay + az * az;

	// Compute feedback only if accelerometer abs(vector) less than cutoff value (also avoids NaN in accelerometer normalisation)
	if( fabs(sqrt(accelSquareSum) - 9.8065) < accelCutoff )
	{
		// Normalise accelerometer measurement
		recipNorm = inv_sqrt(accelSquareSum);

		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Normalise magnetometer measurement
		recipNorm = inv_sqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * ahrs->quat.q0 * mx;
		_2q0my = 2.0f * ahrs->quat.q0 * my;
		_2q0mz = 2.0f * ahrs->quat.q0 * mz;
		_2q1mx = 2.0f * ahrs->quat.q1 * mx;
		_2q0   = 2.0f * ahrs->quat.q0;
		_2q1   = 2.0f * ahrs->quat.q1;
		_2q2   = 2.0f * ahrs->quat.q2;
		_2q3   = 2.0f * ahrs->quat.q3;
		_2q0q2 = 2.0f * ahrs->quat.q0 * ahrs->quat.q2;
		_2q2q3 = 2.0f * ahrs->quat.q2 * ahrs->quat.q3;
		q0q0   =   ahrs->quat.q0 * ahrs->quat.q0;
		q0q1   =   ahrs->quat.q0 * ahrs->quat.q1;
		q0q2   =   ahrs->quat.q0 * ahrs->quat.q2;
		q0q3   =   ahrs->quat.q0 * ahrs->quat.q3;
		q1q1   =   ahrs->quat.q1 * ahrs->quat.q1;
		q1q2   =   ahrs->quat.q1 * ahrs->quat.q2;
		q1q3   =   ahrs->quat.q1 * ahrs->quat.q3;
		q2q2   =   ahrs->quat.q2 * ahrs->quat.q2;
		q2q3   =   ahrs->quat.q2 * ahrs->quat.q3;
		q3q3   =   ahrs->quat.q3 * ahrs->quat.q3;

		// Reference direction of Earth's magnetic field
		hx   = mx * q0q0 - _2q0my * ahrs->quat.q3 + _2q0mz * ahrs->quat.q2 + mx * q1q1 + _2q1 * my * ahrs->quat.q2 + _2q1 * mz * ahrs->quat.q3 - mx * q2q2 - mx * q3q3;
		hy   = _2q0mx * ahrs->quat.q3 + my * q0q0 - _2q0mz * ahrs->quat.q1 + _2q1mx * ahrs->quat.q2 - my * q1q1 + my * q2q2 + _2q2 * mz * ahrs->quat.q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * ahrs->quat.q2 + _2q0my * ahrs->quat.q1 + mz * q0q0 + _2q1mx * ahrs->quat.q3 - mz * q1q1 + _2q2 * my * ahrs->quat.q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 + ax) + _2q1 * (2.0f * q0q1 + _2q2q3 + ay) - _2bz * ahrs->quat.q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * ahrs->quat.q3 + _2bz * ahrs->quat.q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * ahrs->quat.q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 =  _2q3 * (2.0f * q1q3 - _2q0q2 + ax) + _2q0 * (2.0f * q0q1 + _2q2q3 + ay) - 4.0f * ahrs->quat.q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 + az) + _2bz * ahrs->quat.q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * ahrs->quat.q2 + _2bz * ahrs->quat.q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * ahrs->quat.q3 - _4bz * ahrs->quat.q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 + ax) + _2q3 * (2.0f * q0q1 + _2q2q3 + ay) - 4.0f * ahrs->quat.q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 + az) + (-_4bx * ahrs->quat.q2 - _2bz * ahrs->quat.q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * ahrs->quat.q1 + _2bz * ahrs->quat.q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * ahrs->quat.q0 - _4bz * ahrs->quat.q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 =  _2q1 * (2.0f * q1q3 - _2q0q2 + ax) + _2q2 * (2.0f * q0q1 + _2q2q3 + ay) + (-_4bx * ahrs->quat.q3 + _2bz * ahrs->quat.q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * ahrs->quat.q0 + _2bz * ahrs->quat.q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * ahrs->quat.q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

		recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot0 -= ahrs->beta * s0;
		qDot1 -= ahrs->beta * s1;
		qDot2 -= ahrs->beta * s2;
		qDot3 -= ahrs->beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	ahrs->quat.q0 += qDot0 * dt;
	ahrs->quat.q1 += qDot1 * dt;
	ahrs->quat.q2 += qDot2 * dt;
	ahrs->quat.q3 += qDot3 * dt;

	// Normalise quaternion
	recipNorm = inv_sqrt(ahrs->quat.q0 * ahrs->quat.q0 + ahrs->quat.q1 * ahrs->quat.q1 + ahrs->quat.q2 * ahrs->quat.q2 + ahrs->quat.q3 * ahrs->quat.q3);
	ahrs->quat.q0 *= recipNorm;
	ahrs->quat.q1 *= recipNorm;
	ahrs->quat.q2 *= recipNorm;
	ahrs->quat.q3 *= recipNorm;
}

