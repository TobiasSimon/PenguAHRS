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

#include "util.h"
#include "madgwick_ahrs.h"


void madgwick_ahrs_init(madgwick_ahrs_t *ahrs, float ax, float ay, float az, float mx, float my, float mz, float beta)
{
   float initialRoll, initialPitch;
   float cosRoll, sinRoll, cosPitch, sinPitch;
   float magX, magY;
   float initialHdg, cosHeading, sinHeading;

   initialRoll  = atan2( -ay, -az );
   initialPitch = atan2(  ax, -az );

   cosRoll =  cosf( initialRoll  );
   sinRoll =  sinf( initialRoll  );
   cosPitch = cosf( initialPitch );
   sinPitch = sinf( initialPitch );

   magX = mx * cosPitch + my * sinRoll * sinPitch + mz * cosRoll * sinPitch;

   magY = my * cosRoll - mz * sinRoll;

   initialHdg = atan2( -magY, magX );

   cosRoll =  cosf( initialRoll * 0.5f  );
   sinRoll =  sinf( initialRoll * 0.5f  );

   cosPitch = cosf( initialPitch * 0.5f );
   sinPitch = sinf( initialPitch * 0.5f );

   cosHeading = cosf( initialHdg * 0.5f );
   sinHeading = sinf( initialHdg * 0.5f );

   ahrs->q0 = cosRoll * cosPitch * cosHeading + sinRoll * sinPitch * sinHeading;
   ahrs->q1 = sinRoll * cosPitch * cosHeading - cosRoll * sinPitch * sinHeading;
   ahrs->q2 = cosRoll * sinPitch * cosHeading + sinRoll * cosPitch * sinHeading;
   ahrs->q3 = cosRoll * cosPitch * sinHeading - sinRoll * sinPitch * cosHeading;
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
	qDot0 = 0.5f * (-ahrs->q1 * gx - ahrs->q2 * gy - ahrs->q3 * gz);
	qDot1 = 0.5f * ( ahrs->q0 * gx + ahrs->q2 * gz - ahrs->q3 * gy);
	qDot2 = 0.5f * ( ahrs->q0 * gy - ahrs->q1 * gz + ahrs->q3 * gx);
	qDot3 = 0.5f * ( ahrs->q0 * gz + ahrs->q1 * gy - ahrs->q2 * gx);

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
		_2q0 = 2.0f * ahrs->q0;
		_2q1 = 2.0f * ahrs->q1;
		_2q2 = 2.0f * ahrs->q2;
		_2q3 = 2.0f * ahrs->q3;
		_4q0 = 4.0f * ahrs->q0;
		_4q1 = 4.0f * ahrs->q1;
		_4q2 = 4.0f * ahrs->q2;
		_8q1 = 8.0f * ahrs->q1;
		_8q2 = 8.0f * ahrs->q2;
		q0q0 = ahrs->q0 * ahrs->q0;
		q1q1 = ahrs->q1 * ahrs->q1;
		q2q2 = ahrs->q2 * ahrs->q2;
		q3q3 = ahrs->q3 * ahrs->q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 - _2q2 * ax + _4q0 * q1q1 + _2q1 * ay;
		s1 = _4q1 * q3q3 + _2q3 * ax + 4.0f * q0q0 * ahrs->q1 + _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 - _4q1 * az;
		s2 = 4.0f * q0q0 * ahrs->q2 - _2q0 * ax + _4q2 * q3q3 + _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 - _4q2 * az;
		s3 = 4.0f * q1q1 * ahrs->q3 + _2q1 * ax + 4.0f * q2q2 * ahrs->q3 + _2q2 * ay;

		recipNorm = inv_sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		// qDot0 -= sensorConfig.beta * s0;
		// qDot1 -= sensorConfig.beta * s1;
		// qDot2 -= sensorConfig.beta * s2;
		// qDot3 -= sensorConfig.beta * s3;
	}

	// Apply feedback step
	qDot0 -= ahrs->beta * s0;
	qDot1 -= ahrs->beta * s1;
	qDot2 -= ahrs->beta * s2;
	qDot3 -= ahrs->beta * s3;

	// Integrate rate of change of quaternion to yield quaternion
	ahrs->q0 += qDot0 * dt;
	ahrs->q1 += qDot1 * dt;
	ahrs->q2 += qDot2 * dt;
	ahrs->q3 += qDot3 * dt;

	// Normalise quaternion
	recipNorm = inv_sqrt(ahrs->q0 * ahrs->q0 + ahrs->q1 * ahrs->q1 + ahrs->q2 * ahrs->q2 + ahrs->q3 * ahrs->q3);
	ahrs->q0 *= recipNorm;
	ahrs->q1 *= recipNorm;
	ahrs->q2 *= recipNorm;
	ahrs->q3 *= recipNorm;
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
	qDot0 = 0.5f * (-ahrs->q1 * gx - ahrs->q2 * gy - ahrs->q3 * gz);
	qDot1 = 0.5f * ( ahrs->q0 * gx + ahrs->q2 * gz - ahrs->q3 * gy);
	qDot2 = 0.5f * ( ahrs->q0 * gy - ahrs->q1 * gz + ahrs->q3 * gx);
	qDot3 = 0.5f * ( ahrs->q0 * gz + ahrs->q1 * gy - ahrs->q2 * gx);

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
		_2q0mx = 2.0f * ahrs->q0 * mx;
		_2q0my = 2.0f * ahrs->q0 * my;
		_2q0mz = 2.0f * ahrs->q0 * mz;
		_2q1mx = 2.0f * ahrs->q1 * mx;
		_2q0   = 2.0f * ahrs->q0;
		_2q1   = 2.0f * ahrs->q1;
		_2q2   = 2.0f * ahrs->q2;
		_2q3   = 2.0f * ahrs->q3;
		_2q0q2 = 2.0f * ahrs->q0 * ahrs->q2;
		_2q2q3 = 2.0f * ahrs->q2 * ahrs->q3;
		q0q0   =   ahrs->q0 * ahrs->q0;
		q0q1   =   ahrs->q0 * ahrs->q1;
		q0q2   =   ahrs->q0 * ahrs->q2;
		q0q3   =   ahrs->q0 * ahrs->q3;
		q1q1   =   ahrs->q1 * ahrs->q1;
		q1q2   =   ahrs->q1 * ahrs->q2;
		q1q3   =   ahrs->q1 * ahrs->q3;
		q2q2   =   ahrs->q2 * ahrs->q2;
		q2q3   =   ahrs->q2 * ahrs->q3;
		q3q3   =   ahrs->q3 * ahrs->q3;

		// Reference direction of Earth's magnetic field
		hx   = mx * q0q0 - _2q0my * ahrs->q3 + _2q0mz * ahrs->q2 + mx * q1q1 + _2q1 * my * ahrs->q2 + _2q1 * mz * ahrs->q3 - mx * q2q2 - mx * q3q3;
		hy   = _2q0mx * ahrs->q3 + my * q0q0 - _2q0mz * ahrs->q1 + _2q1mx * ahrs->q2 - my * q1q1 + my * q2q2 + _2q2 * mz * ahrs->q3 - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * ahrs->q2 + _2q0my * ahrs->q1 + mz * q0q0 + _2q1mx * ahrs->q3 - mz * q1q1 + _2q2 * my * ahrs->q3 - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;

		// Gradient decent algorithm corrective step
		s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 + ax) + _2q1 * (2.0f * q0q1 + _2q2q3 + ay) - _2bz * ahrs->q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * ahrs->q3 + _2bz * ahrs->q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * ahrs->q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s1 =  _2q3 * (2.0f * q1q3 - _2q0q2 + ax) + _2q0 * (2.0f * q0q1 + _2q2q3 + ay) - 4.0f * ahrs->q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 + az) + _2bz * ahrs->q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * ahrs->q2 + _2bz * ahrs->q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * ahrs->q3 - _4bz * ahrs->q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 + ax) + _2q3 * (2.0f * q0q1 + _2q2q3 + ay) - 4.0f * ahrs->q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 + az) + (-_4bx * ahrs->q2 - _2bz * ahrs->q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * ahrs->q1 + _2bz * ahrs->q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * ahrs->q0 - _4bz * ahrs->q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
		s3 =  _2q1 * (2.0f * q1q3 - _2q0q2 + ax) + _2q2 * (2.0f * q0q1 + _2q2q3 + ay) + (-_4bx * ahrs->q3 + _2bz * ahrs->q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * ahrs->q0 + _2bz * ahrs->q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * ahrs->q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);

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
	ahrs->q0 += qDot0 * dt;
	ahrs->q1 += qDot1 * dt;
	ahrs->q2 += qDot2 * dt;
	ahrs->q3 += qDot3 * dt;

	// Normalise quaternion
	recipNorm = inv_sqrt(ahrs->q0 * ahrs->q0 + ahrs->q1 * ahrs->q1 + ahrs->q2 * ahrs->q2 + ahrs->q3 * ahrs->q3);
	ahrs->q0 *= recipNorm;
	ahrs->q1 *= recipNorm;
	ahrs->q2 *= recipNorm;
	ahrs->q3 *= recipNorm;
}

