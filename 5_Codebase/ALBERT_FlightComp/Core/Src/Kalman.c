/*
 *
 * 6-DOF strapdown EKF
 *
 *  state  x = [u v w phi theta psi]
 *  input  u = [ax ay az p q r]
 *
 * Frame: nav-x is up, g_nav = [-g, 0, 0].
 * Upright rest pose: all angles zero, accelerometer reads [+g, 0, 0],
 * phi is heading (axial rotation about body-x).
 *
 * Author: Felix Zauner
 */

#include "Kalman.h"
#include <math.h>
#include <string.h>
#include <stdbool.h>

//													:
//												   :::
//							'::                   ::::
//							'::::.     .....:::.:::::::
//							'::::::::::::::::::::::::::::
//							::::::XUWWWWWU:::::XW$$$$$$WX:
//							::::X$$$$$$$$$$W::X$$$$$$$$$$Wh
//						   ::::t$$$$$$$$$$$$W:$$$$$$P*$$$$M::
//						   :::X$$$$$$""""$$$$X$$$$$   ^$$$$X:::
//						  ::::M$$$$$$    ^$$$RM$$$L    <$$$X::::
//						.:::::M$$$$$$     $$$R:$$$$.   d$$R:::`
//					   '~::::::?$$$$$$...d$$$X$6R$$$$$$$$RXW$X:'`
//						 '~:WNWUXT#$$$$$$$$TU$$$$W6IBBIW@$$RX:
//						  `~<RXX$$$$$$$$$$$F       3$$$$$$T@"
//							 '#$XR$$$$$$$$$L      .$$$$$#Xf
//							   '"*WZ*$$$$$$$$$$$$$$$RR2@"
//								  '""RNH8IIIIIIIMMH??(.
//								 ..:::::::::::::::::::::::::..
//						   .:::::::::::::::::::::::::::::::::::::::.         :::::
//		 .::.         .::::::::::::::'``::::::::::::::::::::::::::::::::::..:::::'
//		:::::::::::::::::::::::''`     :::::::::::::::::::'```::::::::::::::::'`
//		 ``''::::::::::.`::'`          ::::::::::::::::::'      `::::::::::::
//			 ::::::::::::              :::::::::::::::::         '::::::::::::
//			::::::::::::                ::::::::::::::::          `::::::::::`:
//		   :::::::::::::                 :::::::::::::::           `::::::::: :
//		   : ::::::::::                 :::::::::::::::             '::::: :::
//		   :::::'::::::                 :::::::: ::::::               `::::':'
//			:::.'::::'                 ::::::::: ::::::
//			`:::'''`                   :::::::::'::::::
//			 `''                      :::::::::'::::::'
//				.:::::                ::::::::: ::::::
//			  :::::::::::.       .:: :::::::::.:::::::
//			  '::::::::::::::::::::: :::::::::.:::::::
//				`::::::::'``..:::::..::::::::: :::::::::::::::.
//				  ``':: :::::::::::::::::::::: ::::::::::::::::::
//					   ::::::::::::::::::::::: :::::::::::::::::::
//					  '::::::::::::::::::::::' `::::::::::::::::'
//					   `'::::::::::::::::''`          ``````
//							````````

/* ============================================================
 *   Small linear-algebra helpers (row-major, fixed sizes)
 * ============================================================ */

static void mat_zero(float *A, int rows, int cols)
{
    memset(A, 0, sizeof(float) * rows * cols);
}

static void mat_eye(float *A, int n)
{
    mat_zero(A, n, n);
    for (int i = 0; i < n; i++) A[i*n + i] = 1.0f;
}

/* C(rA x cB) = A(rA x cA) * B(cA x cB) */
static void mat_mul(const float *A, const float *B, float *C,
                    int rA, int cA, int cB)
{
    for (int i = 0; i < rA; i++) {
        for (int j = 0; j < cB; j++) {
            float s = 0.0f;
            for (int k = 0; k < cA; k++) s += A[i*cA + k] * B[k*cB + j];
            C[i*cB + j] = s;
        }
    }
}

/* C = A*B^T   A(rA x cA), B(rB x cA) -> C(rA x rB) */
static void mat_mul_bt(const float *A, const float *B, float *C,
                       int rA, int cA, int rB)
{
    for (int i = 0; i < rA; i++) {
        for (int j = 0; j < rB; j++) {
            float s = 0.0f;
            for (int k = 0; k < cA; k++) s += A[i*cA + k] * B[j*cA + k];
            C[i*rB + j] = s;
        }
    }
}

static void mat_add(float *A, const float *B, int n)
{
    for (int i = 0; i < n; i++) A[i] += B[i];
}

static void mat_sub(float *A, const float *B, int n)
{
    for (int i = 0; i < n; i++) A[i] -= B[i];
}

static void mat_scal(float *A, float s, int n)
{
    for (int i = 0; i < n; i++) A[i] *= s;
}

/* Force symmetry of a square n x n matrix (numerical hygiene). */
static void mat_sym(float *A, int n)
{
    for (int i = 0; i < n; i++) {
        for (int j = i+1; j < n; j++) {
            float v = 0.5f * (A[i*n + j] + A[j*n + i]);
            A[i*n + j] = v;
            A[j*n + i] = v;
        }
    }
}

static inline bool inv3x3(const float M[9], float invOut[9])
{
    const float m00=M[0], m01=M[1], m02=M[2];
    const float m10=M[3], m11=M[4], m12=M[5];
    const float m20=M[6], m21=M[7], m22=M[8];

    const float c00 =  (m11*m22 - m12*m21);
    const float c01 = -(m10*m22 - m12*m20);
    const float c02 =  (m10*m21 - m11*m20);
    const float c10 = -(m01*m22 - m02*m21);
    const float c11 =  (m00*m22 - m02*m20);
    const float c12 = -(m00*m21 - m01*m20);
    const float c20 =  (m01*m12 - m02*m11);
    const float c21 = -(m00*m12 - m02*m10);
    const float c22 =  (m00*m11 - m01*m10);

    const float det = m00*c00 + m01*c01 + m02*c02;
    if (fabsf(det) < 1e-9f) return false;
    const float invDet = 1.0f / det;

    invOut[0]=c00*invDet; invOut[1]=c10*invDet; invOut[2]=c20*invDet;
    invOut[3]=c01*invDet; invOut[4]=c11*invDet; invOut[5]=c21*invDet;
    invOut[6]=c02*invDet; invOut[7]=c12*invDet; invOut[8]=c22*invDet;
    return true;
}


/* ============================================================
 *   Rotation  body <- nav   for  R * [-g,0,0]  and  R^T * v_b
 * ============================================================ */

static void body_from_nav(float phi, float theta, float psi, float R[9])
{
    const float sp = sinf(phi),   cp = cosf(phi);
    const float st = sinf(theta), ct = cosf(theta);
    const float sy = sinf(psi),   cy = cosf(psi);

    /* R = Rx(phi) * Ry(theta) * Rz(psi) */
    R[0] =  ct*cy;
    R[1] =  ct*sy;
    R[2] = -st;

    R[3] = -cp*sy + sp*st*cy;
    R[4] =  cp*cy + sp*st*sy;
    R[5] =  sp*ct;

    R[6] =  sp*sy + cp*st*cy;
    R[7] = -sp*cy + cp*st*sy;
    R[8] =  cp*ct;
}


/* ============================================================
 *   Init
 * ============================================================ */

void Kalman_Init(Kalman *kal, float Pinit, const float *Q,
                 const float *R_accel, float R_mag, float R_baro,
                 const float *R_gps)
{
    for (int i = 0; i < KAL_N;  i++) kal->x[i] = 0.0f;

    mat_eye(kal->P, KAL_N);
    mat_scal(kal->P, Pinit, KAL_NN);

    for (int i = 0; i < KAL_N; i++) kal->Q[i] = Q[i];

    kal->R_accel[0] = R_accel[0];
    kal->R_accel[1] = R_accel[1];
    kal->R_accel[2] = R_accel[2];
    kal->R_mag  = R_mag;
    kal->R_baro = R_baro;
    kal->R_gps[0] = R_gps[0];
    kal->R_gps[1] = R_gps[1];
    kal->R_gps[2] = R_gps[2];
}


/* ============================================================
 *   Predict:   x_{k+1} = x_k + dt * f(x,u)
 *              P_{k+1} = Phi P Phi^T + Q*dt ,  Phi = I + F*dt
 * ============================================================ */

void Kalman_Predict(Kalman *kal, const float accel_mps2[3],
                    const float gyro_rdps[3], float dt)
{
    const float u     = kal->x[0], v     = kal->x[1], w     = kal->x[2];
    const float phi   = kal->x[3], theta = kal->x[4], psi   = kal->x[5];

    const float ax = accel_mps2[0], ay = accel_mps2[1], az = accel_mps2[2];
    const float p  = gyro_rdps[0],  q  = gyro_rdps[1],  r  = gyro_rdps[2];

    const float sp = sinf(phi),   cp = cosf(phi);
    const float st = sinf(theta), ct = cosf(theta);
    const float sy = sinf(psi),   cy = cosf(psi);

    /* keep tan(theta) / 1/cos(theta) away from the singularity at +-90 deg */
    const float ct_safe = (fabsf(ct) > 1e-4f) ? ct : (ct >= 0.0f ? 1e-4f : -1e-4f);
    const float tt      = st / ct_safe;
    const float sec     = 1.0f / ct_safe;
    const float sec2    = sec * sec;

    /* ---- f(x,u) : gravity in body + coriolis (v_dot = a + g_b - w x v) ---- */
    const float gbx = -g *  ct*cy;
    const float gby =  g * (cp*sy - sp*st*cy);
    const float gbz = -g * (sp*sy + cp*st*cy);

    const float f_u = ax + gbx - (q*w - r*v);
    const float f_v = ay + gby - (r*u - p*w);
    const float f_w = az + gbz - (p*v - q*u);

    const float f_phi   =  p + (q*sp + r*cp) * tt;
    const float f_theta =      q*cp - r*sp;
    const float f_psi   =     (q*sp + r*cp) * sec;

    /* ---- Jacobian F = df/dx  (6x6 row-major) ---- */
    float F[KAL_NN]; mat_zero(F, KAL_N, KAL_N);
    #define Fm(i,j) F[(i)*KAL_N + (j)]

    /* row u: a_x - g*ct*cy - q*w + r*v   */
    Fm(0,1) =  r;
    Fm(0,2) = -q;
    Fm(0,4) =  g * st * cy;      /* d/dtheta  */
    Fm(0,5) =  g * ct * sy;      /* d/dpsi    */

    /* row v: a_y + g(cp*sy - sp*st*cy) - r*u + p*w */
    Fm(1,0) = -r;
    Fm(1,2) =  p;
    Fm(1,3) = -g * (sp*sy + cp*st*cy);   /* d/dphi   */
    Fm(1,4) = -g *  sp*ct*cy;            /* d/dtheta */
    Fm(1,5) =  g * (cp*cy + sp*st*sy);   /* d/dpsi   */

    /* row w: a_z - g(sp*sy + cp*st*cy) - p*v + q*u */
    Fm(2,0) =  q;
    Fm(2,1) = -p;
    Fm(2,3) =  g * (sp*st*cy - cp*sy);   /* d/dphi   */
    Fm(2,4) = -g *  cp*ct*cy;            /* d/dtheta */
    Fm(2,5) = -g * (sp*cy - cp*st*sy);   /* d/dpsi   */

    /* row phi: p + (q*sp + r*cp)*tan(theta) */
    Fm(3,3) = (q*cp - r*sp) * tt;
    Fm(3,4) = (q*sp + r*cp) * sec2;

    /* row theta: q*cp - r*sp */
    Fm(4,3) = -(q*sp + r*cp);

    /* row psi: (q*sp + r*cp)/cos(theta) */
    Fm(5,3) = (q*cp - r*sp) * sec;
    Fm(5,4) = (q*sp + r*cp) * tt * sec;
    #undef Fm

    /* ---- integrate state (forward Euler) ---- */
    kal->x[0] += dt * f_u;
    kal->x[1] += dt * f_v;
    kal->x[2] += dt * f_w;
    kal->x[3] += dt * f_phi;
    kal->x[4] += dt * f_theta;
    kal->x[5] += dt * f_psi;

    /* ---- Phi = I + F*dt ---- */
    float Phi[KAL_NN];
    mat_eye(Phi, KAL_N);
    for (int i = 0; i < KAL_NN; i++) Phi[i] += dt * F[i];

    /* ---- P <- Phi * P * Phi^T + Q*dt ---- */
    float tmp[KAL_NN];
    mat_mul(Phi, kal->P, tmp, KAL_N, KAL_N, KAL_N);    /* tmp = Phi * P   */
    mat_mul_bt(tmp, Phi, kal->P, KAL_N, KAL_N, KAL_N); /* P = tmp * Phi^T */

    for (int i = 0; i < KAL_N; i++) kal->P[i*KAL_N + i] += kal->Q[i] * dt;

    mat_sym(kal->P, KAL_N);
}


/* ============================================================
 *   Generic EKF measurement update (Joseph form)
 *
 *      y  : innovation   v = z - h(x)                (m x 1)
 *      H  : Jacobian     dh/dx at x                  (m x KAL_N)
 *      R  : meas noise diagonal                      (m)
 *
 *   Used internally by the public Update* functions.
 * ============================================================ */

static void ekf_update(Kalman *kal, const float *y, const float *H,
                       const float *Rdiag, int m)
{
    /* PH^T  (KAL_N x m)  */
    float PHt[KAL_N * 3];   /* m <= 3 for all sensors here */
    for (int i = 0; i < KAL_N; i++) {
        for (int j = 0; j < m; j++) {
            float s = 0.0f;
            for (int k = 0; k < KAL_N; k++) s += kal->P[i*KAL_N + k] * H[j*KAL_N + k];
            PHt[i*m + j] = s;
        }
    }

    /* S = H * PH^T + R   (m x m) */
    float S[9];
    for (int i = 0; i < m; i++) {
        for (int j = 0; j < m; j++) {
            float s = 0.0f;
            for (int k = 0; k < KAL_N; k++) s += H[i*KAL_N + k] * PHt[k*m + j];
            S[i*m + j] = s + (i == j ? Rdiag[i] : 0.0f);
        }
    }

    /* K = PH^T * S^-1   (KAL_N x m) */
    float Sinv[9];
    if (m == 1) {
        if (fabsf(S[0]) < 1e-12f) return;
        Sinv[0] = 1.0f / S[0];
    } else if (m == 3) {
        if (!inv3x3(S, Sinv)) return;
    } else {
        return;
    }

    float K[KAL_N * 3];
    for (int i = 0; i < KAL_N; i++) {
        for (int j = 0; j < m; j++) {
            float s = 0.0f;
            for (int k = 0; k < m; k++) s += PHt[i*m + k] * Sinv[k*m + j];
            K[i*m + j] = s;
        }
    }

    /* x <- x + K*y */
    for (int i = 0; i < KAL_N; i++) {
        float s = 0.0f;
        for (int j = 0; j < m; j++) s += K[i*m + j] * y[j];
        kal->x[i] += s;
    }

    /* Joseph form:  P = (I-KH) P (I-KH)^T + K R K^T   */
    float IKH[KAL_NN];
    mat_eye(IKH, KAL_N);
    for (int i = 0; i < KAL_N; i++) {
        for (int j = 0; j < KAL_N; j++) {
            float s = 0.0f;
            for (int k = 0; k < m; k++) s += K[i*m + k] * H[k*KAL_N + j];
            IKH[i*KAL_N + j] -= s;
        }
    }

    float tmp[KAL_NN];
    mat_mul   (IKH, kal->P, tmp,     KAL_N, KAL_N, KAL_N);    /* tmp = (I-KH) P      */
    mat_mul_bt(tmp, IKH,    kal->P,  KAL_N, KAL_N, KAL_N);    /* P  = tmp * (I-KH)^T */

    /* + K R K^T */
    for (int i = 0; i < KAL_N; i++) {
        for (int j = 0; j < KAL_N; j++) {
            float s = 0.0f;
            for (int k = 0; k < m; k++) s += K[i*m + k] * Rdiag[k] * K[j*m + k];
            kal->P[i*KAL_N + j] += s;
        }
    }

    mat_sym(kal->P, KAL_N);
}


/* ============================================================
 *   Accelerometer:  specific-force direction gives theta, psi
 *                   (gated to |a| ~ g so boost acceleration is rejected)
 * ============================================================ */

void Kalman_UpdateAccel(Kalman *kal, const float accel_mps2[3])
{
    const float ax = accel_mps2[0];
    const float ay = accel_mps2[1];
    const float az = accel_mps2[2];

    const float an = sqrtf(ax*ax + ay*ay + az*az);
    if (an < 1e-6f) return;

    /* gate: during thrust |a| != g -> skip this update */
    const float gate = 0.05f * g;
    if (fabsf(an - g) > gate) return;

    const float phi   = kal->x[3];
    const float theta = kal->x[4];
    const float psi   = kal->x[5];

    const float sp = sinf(phi),   cp = cosf(phi);
    const float st = sinf(theta), ct = cosf(theta);
    const float sy = sinf(psi),   cy = cosf(psi);

    /* predicted specific force at rest (gravity direction in body, negated) */
    const float h0 = g *  ct*cy;
    const float h1 = g * (sp*st*cy - cp*sy);
    const float h2 = g * (sp*sy    + cp*st*cy);

    const float yv[3] = { ax - h0, ay - h1, az - h2 };

    /* H = dh/dx  (3 x 6).  Columns for u,v,w are zero. */
    float H[3 * KAL_N];
    mat_zero(H, 3, KAL_N);
    #define Hm(i,j) H[(i)*KAL_N + (j)]

    /* row 0: g*ct*cy */
    Hm(0,4) = -g * st * cy;      /* d/dtheta */
    Hm(0,5) = -g * ct * sy;      /* d/dpsi   */

    /* row 1: g*(sp*st*cy - cp*sy) */
    Hm(1,3) =  g * (cp*st*cy + sp*sy);
    Hm(1,4) =  g *  sp*ct*cy;
    Hm(1,5) = -g * (sp*st*sy + cp*cy);

    /* row 2: g*(sp*sy + cp*st*cy) */
    Hm(2,3) =  g * (cp*sy - sp*st*cy);
    Hm(2,4) =  g *  cp*ct*cy;
    Hm(2,5) =  g * (sp*cy - cp*st*sy);
    #undef Hm

    ekf_update(kal, yv, H, kal->R_accel, 3);
}


/* ============================================================
 *   Magnetometer:  measures phi (heading, axial orientation)
 * ============================================================ */

void Kalman_UpdateMag(Kalman *kal, float phi_meas)
{
    static const float PI_F = 3.14159265358979323846f;

    /* wrap innovation to (-pi, pi] so a transition across +-pi doesn't blow up */
    float err = phi_meas - kal->x[3];
    while (err >   PI_F) err -= 2.0f * PI_F;
    while (err <= -PI_F) err += 2.0f * PI_F;

    float H[KAL_N];
    mat_zero(H, 1, KAL_N);
    H[3] = 1.0f;

    ekf_update(kal, &err, H, &kal->R_mag, 1);
}


/* ============================================================
 *   Barometer:  vertical velocity -> state u  (body-x = up when upright)
 *
 *   Caller passes dh/dt (filtered altitude derivative, positive up).
 *   h_baro(x) = u.
 * ============================================================ */

void Kalman_UpdateBaro(Kalman *kal, float u_meas)
{
    const float err = u_meas - kal->x[0];

    float H[KAL_N];
    mat_zero(H, 1, KAL_N);
    H[0] = 1.0f;

    ekf_update(kal, &err, H, &kal->R_baro, 1);
}


/* ============================================================
 *   GPS:  nav-frame velocity -> transforms body [u,v,w] to nav
 *
 *   h_gps(x) = R_{n<-b} * [u,v,w] = R_{b<-n}^T * [u,v,w].
 *   Not called yet - wire in when the u-blox driver is alive.
 * ============================================================ */

void Kalman_UpdateGPS(Kalman *kal, const float vel_nav_mps[3])
{
    const float u     = kal->x[0], v     = kal->x[1], w     = kal->x[2];
    const float phi   = kal->x[3], theta = kal->x[4], psi   = kal->x[5];

    float R[9];
    body_from_nav(phi, theta, psi, R);  /* R_{b<-n}; R^T is R_{n<-b} */

    /* h = R^T * [u,v,w] */
    const float h0 = R[0]*u + R[3]*v + R[6]*w;
    const float h1 = R[1]*u + R[4]*v + R[7]*w;
    const float h2 = R[2]*u + R[5]*v + R[8]*w;

    const float yv[3] = { vel_nav_mps[0] - h0,
                          vel_nav_mps[1] - h1,
                          vel_nav_mps[2] - h2 };

    const float sp = sinf(phi),   cp = cosf(phi);
    const float st = sinf(theta), ct = cosf(theta);
    const float sy = sinf(psi),   cy = cosf(psi);

    /* h_i = (column i of R) . [u,v,w]                                          *
     *                                                                           *
     *  col 0 of R = [  ct*cy       , -cp*sy + sp*st*cy ,  sp*sy + cp*st*cy ]^T  *
     *  col 1 of R = [  ct*sy       ,  cp*cy + sp*st*sy , -sp*cy + cp*st*sy ]^T  *
     *  col 2 of R = [ -st          ,  sp*ct            ,  cp*ct            ]^T  */

    float H[3 * KAL_N];
    mat_zero(H, 3, KAL_N);
    #define Hm(i,j) H[(i)*KAL_N + (j)]

    /* d h / d[u,v,w] = rows of R^T = columns of R */
    Hm(0,0) = R[0]; Hm(0,1) = R[3]; Hm(0,2) = R[6];
    Hm(1,0) = R[1]; Hm(1,1) = R[4]; Hm(1,2) = R[7];
    Hm(2,0) = R[2]; Hm(2,1) = R[5]; Hm(2,2) = R[8];

    /* d h0 / d(phi, theta, psi) */
    Hm(0,3) = ( sp*sy + cp*st*cy) * v + ( cp*sy - sp*st*cy) * w;
    Hm(0,4) = (-st*cy           ) * u + ( sp*ct*cy        ) * v + ( cp*ct*cy) * w;
    Hm(0,5) = (-ct*sy           ) * u + (-cp*cy - sp*st*sy) * v + ( sp*cy - cp*st*sy) * w;

    /* d h1 / d(phi, theta, psi) */
    Hm(1,3) = (-sp*cy + cp*st*sy) * v + (-cp*cy - sp*st*sy) * w;
    Hm(1,4) = (-st*sy           ) * u + ( sp*ct*sy        ) * v + ( cp*ct*sy) * w;
    Hm(1,5) = ( ct*cy           ) * u + (-cp*sy + sp*st*cy) * v + ( sp*sy + cp*st*cy) * w;

    /* d h2 / d(phi, theta, psi) */
    Hm(2,3) = ( cp*ct) * v + (-sp*ct) * w;
    Hm(2,4) = (-ct)    * u + (-sp*st) * v + (-cp*st) * w;
    Hm(2,5) = 0.0f;
    #undef Hm

    ekf_update(kal, yv, H, kal->R_gps, 3);
}


/* ============================================================
 *   ZUPT:  pull body velocity [u,v,w] toward zero when the
 *          rocket is detected static (gyro quiet and |a| ~ g).
 *
 *   h_zupt(x) = [u,v,w] ;  z = [0,0,0]
 * ============================================================ */

void Kalman_UpdateZUPT(Kalman *kal, float R_zupt)
{
    const float y[3] = { -kal->x[0], -kal->x[1], -kal->x[2] };

    float H[3 * KAL_N];
    mat_zero(H, 3, KAL_N);
    H[0 * KAL_N + 0] = 1.0f;
    H[1 * KAL_N + 1] = 1.0f;
    H[2 * KAL_N + 2] = 1.0f;

    const float Rv[3] = { R_zupt, R_zupt, R_zupt };
    ekf_update(kal, y, H, Rv, 3);
}
