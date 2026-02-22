/*
 *
 * EKF Implementation
 *
 * Author: Felix Zauner
 * Created: 11.02.2026
 *
 */

//
//#ifndef KALMAN_H
//#define KALMAN_H
//
//#ifdef __cplusplus
//extern "C" {
//#endif
//
//#include <stddef.h> // for size_t
//
//
//
//typedef struct {
//    double x[7];         // State: [qw qx qy qz bx by bz]
//    double P[7][7];      // State covariance (7x7)
//    double Q[3][3];      // Gyro noise covariance (3x3)
//    double Q_bias[7][7]; // Gyro bias random walk covariance (7x7, only lower-right 3x3 used)
//    double R[3][3];      // Accelerometer measurement covariance (3x3)
//} EKF;
//
//// Initialize EKF with parameters
//void ekf_init(EKF *ekf,
//              const double q0[4],
//              const double b0[3],
//              double init_gyro_bias_err,     // 1-sigma [rad/s]
//              double gyro_noise,             // 1-sigma [rad/s]
//              double gyro_bias_noise,        // 1-sigma [rad/s]
//              double accelerometer_noise);   // 1-sigma [m/s^2]
//
//// Prediction step (gyro integration).
//// w_meas: gyro measurement in FRD [rad/s]
//// dt: timestep [s]
//void ekf_predict(EKF *ekf, const double w_meas[3], double dt);
//
//// Update step (accelerometer).
//// a_meas: accelerometer measurement in FRD [m/s^2]
//void ekf_update(EKF *ekf, const double a_meas[3]);
//
//#ifdef __cplusplus
//}
//#endif
//
//#endif // EKF_ATTITUDE_H
//
//
//


#ifndef KALMAN_H
#define KALMAN_H

#include <math.h>

#define g ((float) 9.80665f)

typedef struct {
	float phi_rad;
	float theta_rad;

	float P[4];
	float Q[2];
	float R[3];

} Kalman;


void Kalman_Init(Kalman *kal, float Pinit, float *Q, float *R);
void Kalman_Predict(Kalman *kal, float *gyr_rdps, float T);
void Kalman_Update(Kalman *kal, float *acc_mps2);

#endif


