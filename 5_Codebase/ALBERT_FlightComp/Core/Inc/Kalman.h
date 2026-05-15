/*
 *
 * 6-DOF strapdown EKF
 *
 * State:  x = [u, v, w, phi, theta, psi]^T
 *           u,v,w   body-frame velocities      [m/s]
 *           phi     roll / axial spin          [rad]
 *           theta   pitch                      [rad]
 *           psi     yaw                        [rad]
 *
 * Inputs: u = [ax, ay, az, p, q, r]^T
 *           ax,ay,az  specific force (body, accelerometer reading)  [m/s^2]
 *           p,q,r     body rates (gyroscope reading)                [rad/s]
 *
 * Nav frame: x points up, gravity g_nav = [-g, 0, 0]. Upright stationary
 * rocket -> accelerometer reads [+g, 0, 0] and phi is heading.
 *
 * Author: Felix Zauner
 */

#ifndef KALMAN_H
#define KALMAN_H

#include <math.h>
#include <stdint.h>

#define g ((float) 9.80665f)

#define KAL_N     6     /* state dimension                     */
#define KAL_NN    36    /* KAL_N * KAL_N                       */


typedef struct {
    /* state */
    float x[KAL_N];                 /* [u, v, w, phi, theta, psi]      */

    /* covariance (row-major) */
    float P[KAL_NN];

    /* process noise (diagonal) */
    float Q[KAL_N];

    /* measurement noise (diagonal) */
    float R_accel[3];               /* specific force  [ (m/s^2)^2 ]   */
    float R_mag;                    /* phi from mag    [ rad^2 ]       */
    float R_baro;                   /* u from baro     [ (m/s)^2 ]     */
    float R_gps[3];                 /* nav velocity    [ (m/s)^2 ]     */
} Kalman;


void Kalman_Init(Kalman *kal, float Pinit, const float *Q,
                 const float *R_accel, float R_mag, float R_baro,
                 const float *R_gps);

/* Prediction step. accel is specific force in body, gyro is body rates. */
void Kalman_Predict(Kalman *kal, const float accel_mps2[3],
                    const float gyro_rdps[3], float dt);

/* Measurement updates. */
void Kalman_UpdateAccel(Kalman *kal, const float accel_mps2[3]);
void Kalman_UpdateMag  (Kalman *kal, float phi_meas);
void Kalman_UpdateBaro (Kalman *kal, float u_meas);
void Kalman_UpdateGPS  (Kalman *kal, const float vel_nav_mps[3]);

/* Zero-velocity update: force body-frame velocity toward 0.
 * Call only when the rocket is detectably static (e.g. |gyro| below threshold
 * AND |accel| ~ g).  Stops velocity drift on the bench / pre-launch. */
void Kalman_UpdateZUPT (Kalman *kal, float R_zupt);

#endif
