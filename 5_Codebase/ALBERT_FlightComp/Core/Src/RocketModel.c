/*
 * 9-DOF rocket dynamic model.  Port of the MATLAB rocket_f().
 * See RocketModel.h for conventions.
 *
 * Author: Felix Zauner
 */

#include "RocketModel.h"
#include <math.h>
#include <string.h>

/* Lower bound on airspeed V used in divisions (damping terms, asin, atan2). *
 * Below this V, aerodynamic damping / incidence angles are not meaningful.  */
#define RM_V_MIN 0.5f


void RocketModel_DefaultParams(RocketParams *p)
{
    memset(p, 0, sizeof(*p));

    p->g0   = 9.80665f;
    p->rho0 = 1.225f;
    p->S    = 0.010f;
    p->d    = 0.10f;

    p->m  = 1.6f;
    p->Ix = 0.005f;
    p->Iy = 0.020f;
    p->Iz = 0.020f;

    p->CA0       = 0.093292905f;
    p->CA_kalpha = 0.0593605f;
    p->CA_kbeta  = 0.0593605f;

    p->CY_beta  = -2.0577392f;
    p->CY_zeta  = -0.6508215f;
    p->CZ_alpha = -2.0577392f;
    p->CZ_eta   = -0.6508215f;

    p->Cl_delta = 0.229f;
    p->Clp      = -10.0f;

    p->Cm_alpha = 2.6130694f;
    p->Cm_delta = 2.7716625f;
    p->Cmq      = -24.89f;
    p->Cn_beta  = 2.6130694f;
    p->Cn_delta = 2.7716625f;
    p->Cnr      = -24.89f;

    p->xcm  = 0.60f;
    p->xref = 0.50f;
}


void RocketModel_Init(RocketModel *m, const RocketParams *params)
{
    memset(m->x, 0, sizeof(m->x));
    if (params) m->params = *params;
    else        RocketModel_DefaultParams(&m->params);
}


/* --- internal helper: aerodynamic forces in body frame --- */
static void aero_forces(const RocketParams *p, const float x[9],
                        float delta_eta, float delta_zeta,
                        float F_aero[3])
{
    const float ub = x[0], vb = x[1], wb = x[2];

    float V = sqrtf(ub*ub + vb*vb + wb*wb + 1e-12f);
    if (V < RM_V_MIN) V = RM_V_MIN;

    const float alpha = atan2f(wb, ub);

    /* beta = asin(vb / V) — clamp to keep asinf well-defined numerically */
    float sb = vb / V;
    if (sb >  1.0f) sb =  1.0f;
    if (sb < -1.0f) sb = -1.0f;
    const float beta = asinf(sb);

    const float Qdyn = 0.5f * p->rho0 * V * V;

    const float CA = p->CA0 + p->CA_kalpha*alpha + p->CA_kbeta*beta;
    const float CN = p->CY_beta*beta + p->CY_zeta*delta_zeta
                   + p->CZ_alpha*alpha + p->CZ_eta*delta_eta;

    const float side = sqrtf(vb*vb + wb*wb + 1e-9f);
    const float CNy = CN * (-vb) / side;
    const float CNz = CN * (-wb) / side;

    F_aero[0] = -Qdyn * CA  * p->S;
    F_aero[1] =  Qdyn * CNy * p->S;
    F_aero[2] =  Qdyn * CNz * p->S;
}


/* --- internal helper: moment in body frame --- */
static void aero_moments(const RocketParams *p, const float x[9],
                         float delta_eta, float delta_zeta, float delta_r,
                         const float F_aero[3],
                         float M[3])
{
    const float ub = x[0], vb = x[1], wb = x[2];
    const float pr = x[3], qr = x[4], rr = x[5];

    float V = sqrtf(ub*ub + vb*vb + wb*wb + 1e-12f);
    if (V < RM_V_MIN) V = RM_V_MIN;

    const float alpha = atan2f(wb, ub);
    float sb = vb / V;
    if (sb >  1.0f) sb =  1.0f;
    if (sb < -1.0f) sb = -1.0f;
    const float beta = asinf(sb);

    /* CNy, CNz rebuilt from F_aero (same Qdyn*S factor) */
    const float Qdyn = 0.5f * p->rho0 * V * V;
    const float QS   = Qdyn * p->S;
    const float CNy  = (QS > 1e-9f) ? (F_aero[1] / QS) : 0.0f;
    const float CNz  = (QS > 1e-9f) ? (F_aero[2] / QS) : 0.0f;

    const float d_over_2V = p->d / (2.0f * V);
    const float x_over_d  = (p->xcm - p->xref) / p->d;

    const float Cl = p->Cl_delta*delta_r + d_over_2V*p->Clp*pr;
    const float Cm = p->Cm_alpha*alpha + p->Cm_delta*delta_eta
                   - CNz*x_over_d
                   + d_over_2V*(p->Cmq + p->Cm_alpha)*qr;
    const float Cn = p->Cn_beta *beta  + p->Cn_delta*delta_zeta
                   - CNy*x_over_d
                   + d_over_2V*(p->Cnr + p->Cn_beta )*rr;

    const float qSd = Qdyn * p->S * p->d;
    M[0] = qSd * Cl;
    M[1] = qSd * Cm;
    M[2] = qSd * Cn;
}


void RocketModel_Deriv(const RocketModel *m,
                       float delta_eta, float delta_zeta, float delta_r,
                       float thrust_N,
                       float xdot[9])
{
    const RocketParams *p = &m->params;
    const float ub = m->x[0], vb = m->x[1], wb = m->x[2];
    const float pr = m->x[3], qr = m->x[4], rr = m->x[5];
    const float phi = m->x[6], theta = m->x[7], psi = m->x[8];

    /* aerodynamic forces */
    float Fa[3];
    aero_forces(p, m->x, delta_eta, delta_zeta, Fa);

    /* gravity in body frame.  R_n<-b  first column of R_b<-n transposed:
     *   R_b<-n (3-2-1) row 0 = [ ct*cy, ct*sy, -st ]  => that's R_n<-b col 0
     * Fg_b = R_n<-b * [-m g, 0, 0] = -m g * col0 of R_n<-b                  */
    const float sp = sinf(phi),   cp = cosf(phi);
    const float st = sinf(theta), ct = cosf(theta);
    const float sy = sinf(psi),   cy = cosf(psi);

    const float Rnb_col0_x =  ct*cy;
    const float Rnb_col0_y = -cp*sy + sp*st*cy;
    const float Rnb_col0_z =  sp*sy + cp*st*cy;

    const float mg = p->m * p->g0;
    const float Fgx = -mg * Rnb_col0_x;
    const float Fgy = -mg * Rnb_col0_y;
    const float Fgz = -mg * Rnb_col0_z;

    /* total body force */
    const float Fx = Fa[0] + thrust_N + Fgx;
    const float Fy = Fa[1]            + Fgy;
    const float Fz = Fa[2]            + Fgz;

    /* translational */
    xdot[0] = Fx/p->m - (qr*wb - rr*vb);
    xdot[1] = Fy/p->m - (rr*ub - pr*wb);
    xdot[2] = Fz/p->m - (pr*vb - qr*ub);

    /* moments */
    float M[3];
    aero_moments(p, m->x, delta_eta, delta_zeta, delta_r, Fa, M);

    xdot[3] = (M[0] - qr*rr*(p->Iz - p->Iy)) / p->Ix;
    xdot[4] = (M[1] - rr*pr*(p->Ix - p->Iz)) / p->Iy;
    xdot[5] = (M[2] - pr*qr*(p->Iy - p->Ix)) / p->Iz;

    /* Euler kinematics (guard 1/cos(theta)) */
    const float ct_safe = (fabsf(ct) > 1e-4f) ? ct : (ct >= 0.0f ? 1e-4f : -1e-4f);
    const float tt      = st / ct_safe;
    const float sec     = 1.0f / ct_safe;

    xdot[6] = pr + (qr*sp + rr*cp) * tt;
    xdot[7] = qr*cp - rr*sp;
    xdot[8] = (qr*sp + rr*cp) * sec;
}


void RocketModel_Step(RocketModel *m,
                      float delta_eta, float delta_zeta, float delta_r,
                      float thrust_N, float dt)
{
    float xdot[RM_N];
    RocketModel_Deriv(m, delta_eta, delta_zeta, delta_r, thrust_N, xdot);
    for (int i = 0; i < RM_N; i++)
        m->x[i] += xdot[i] * dt;
}


void RocketModel_PredictedAccel(const RocketModel *m,
                                float delta_eta, float delta_zeta,
                                float thrust_N,
                                float a_pred[3])
{
    float Fa[3];
    aero_forces(&m->params, m->x, delta_eta, delta_zeta, Fa);
    const float inv_m = 1.0f / m->params.m;
    a_pred[0] = (Fa[0] + thrust_N) * inv_m;
    a_pred[1] =  Fa[1]             * inv_m;
    a_pred[2] =  Fa[2]             * inv_m;
}
