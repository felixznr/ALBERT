/*
 * 9-DOF rigid-body rocket dynamic model
 *
 *   state  x = [u v w p q r phi theta psi]^T
 *     u,v,w    body-frame velocities    [m/s]
 *     p,q,r    body-frame angular rates [rad/s]
 *     phi,theta,psi  Euler angles (3-2-1), nav-x up
 *
 *   inputs:  delta_eta (pitch fin), delta_zeta (yaw fin), delta_r (roll fin),
 *            thrust_N  (along body-x)
 *
 * This is a PARALLEL reference model, NOT a filter. It is integrated forward
 * from commanded control inputs to provide a physics-based prediction which
 * can be compared against IMU / EKF output in post-flight analysis to
 * validate the aerodynamic coefficients.
 *
 * Frame convention matches Kalman.c (nav-x up, g_nav = [-g, 0, 0]).
 *
 * Author: Felix Zauner
 */

#ifndef ROCKETMODEL_H
#define ROCKETMODEL_H

#include <stdint.h>

#define RM_N 9

typedef struct {
    /* Environment / geometry */
    float g0;          /* gravity            [m/s^2]    */
    float rho0;        /* air density        [kg/m^3]   */
    float S;           /* reference area     [m^2]      */
    float d;           /* reference diameter [m]        */

    /* Mass and inertia */
    float m;
    float Ix, Iy, Iz;

    /* Axial force coefficients */
    float CA0, CA_kalpha, CA_kbeta;

    /* Side / normal force coefficients */
    float CY_beta, CY_zeta;
    float CZ_alpha, CZ_eta;

    /* Moment coefficients */
    float Cl_delta, Clp;
    float Cm_alpha, Cm_delta, Cmq;
    float Cn_beta,  Cn_delta, Cnr;

    /* Reference points */
    float xcm, xref;
} RocketParams;

typedef struct {
    float x[RM_N];
    RocketParams params;
} RocketModel;


/* Fill p with the MATLAB-derived nominal coefficients. */
void RocketModel_DefaultParams(RocketParams *p);

/* Zero state, install params (pass NULL to use defaults). */
void RocketModel_Init(RocketModel *m, const RocketParams *params);

/* xdot = f(x, u) — the right-hand side of the rocket EoM. */
void RocketModel_Deriv(const RocketModel *m,
                       float delta_eta, float delta_zeta, float delta_r,
                       float thrust_N,
                       float xdot[RM_N]);

/* Forward-Euler integrate by dt seconds. */
void RocketModel_Step(RocketModel *m,
                      float delta_eta, float delta_zeta, float delta_r,
                      float thrust_N, float dt);

/* Specific force (body) the accelerometer should read under this model.
 * = (F_aero + F_thrust) / m.   Gravity cancels (accels don't measure g). */
void RocketModel_PredictedAccel(const RocketModel *m,
                                float delta_eta, float delta_zeta,
                                float thrust_N,
                                float a_pred[3]);

#endif
