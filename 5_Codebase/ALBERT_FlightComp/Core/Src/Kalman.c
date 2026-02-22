/*
 *
 * EKF Implementation
 *
 * Author: Felix Zauner
 * Created: 11.02.2026
 *
 */


//
//// ekf_attitude.c
//#include <math.h>
//#include <string.h>
//
//#include "Kalman.h"
//
//
//#define G 9.80665
//
//// ------------------------- small helpers -------------------------
//
//static double vec3_norm(const double v[3]) {
//    return sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
//}
//
//static void vec3_normalize(const double v[3], double out[3]) {
//    double n = vec3_norm(v);
//    if (n > 0.0) {
//        out[0] = v[0] / n;
//        out[1] = v[1] / n;
//        out[2] = v[2] / n;
//    } else {
//        out[0] = out[1] = out[2] = 0.0;
//    }
//}
//
//static void quat_normalize(double q[4]) {
//    double n = sqrt(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);
//    if (n > 0.0) {
//        q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
//    } else {
//        q[0] = 1.0; q[1] = q[2] = q[3] = 0.0;
//    }
//}
//
//static void quat_identity(double q[4]) {
//    q[0] = 1.0; q[1] = 0.0; q[2] = 0.0; q[3] = 0.0;
//}
//
//// Quaternion multiply: r = p ⊗ q
//static void quat_multiply(const double p[4], const double q[4], double r[4]) {
//    const double pw=p[0], px=p[1], py=p[2], pz=p[3];
//    const double qw=q[0], qx=q[1], qy=q[2], qz=q[3];
//    r[0] = pw*qw - px*qx - py*qy - pz*qz;
//    r[1] = pw*qx + px*qw + py*qz - pz*qy;
//    r[2] = pw*qy - px*qz + py*qw + pz*qx;
//    r[3] = pw*qz + px*qy - py*qx + pz*qw;
//}
//
//static void quat_from_axis_angle(const double unit_axis[3], double angle_rad, double q[4]) {
//    double half = 0.5 * angle_rad;
//    double s = sin(half);
//    q[0] = cos(half);
//    q[1] = unit_axis[0] * s;
//    q[2] = unit_axis[1] * s;
//    q[3] = unit_axis[2] * s;
//}
//
//static void quat_from_rotation_vector(const double v[3], double eps, double q[4]) {
//    double angle = vec3_norm(v);
//    if (angle > eps) {
//        double axis[3] = { v[0]/angle, v[1]/angle, v[2]/angle };
//        quat_from_axis_angle(axis, angle, q);
//    } else {
//        quat_identity(q);
//    }
//}
//
//// R = quaternion_to_matrix(q)  (body->world, same as your python)
//static void quat_to_matrix(const double q[4], double R[3][3]) {
//    const double w=q[0], x=q[1], y=q[2], z=q[3];
//    const double w2=w*w, x2=x*x, y2=y*y, z2=z*z;
//
//    R[0][0] = w2 + x2 - y2 - z2;
//    R[0][1] = 2.0*(x*y - w*z);
//    R[0][2] = 2.0*(w*y + x*z);
//
//    R[1][0] = 2.0*(x*y + w*z);
//    R[1][1] = w2 - x2 + y2 - z2;
//    R[1][2] = 2.0*(y*z - w*x);
//
//    R[2][0] = 2.0*(x*z - w*y);
//    R[2][1] = 2.0*(y*z + w*x);
//    R[2][2] = w2 - x2 - y2 + z2;
//}
//
//// ------------------------- matrix ops (fixed sizes) -------------------------
//
//static void mat7_identity(double I[7][7]) {
//    memset(I, 0, sizeof(double)*7*7);
//    for (int i=0;i<7;i++) I[i][i] = 1.0;
//}
//
//static void mat7_transpose(const double A[7][7], double AT[7][7]) {
//    for (int i=0;i<7;i++)
//        for (int j=0;j<7;j++)
//            AT[j][i] = A[i][j];
//}
//
//static void mat3_transpose(const double A[3][3], double AT[3][3]) {
//    for (int i=0;i<3;i++)
//        for (int j=0;j<3;j++)
//            AT[j][i] = A[i][j];
//}
//
//static void mat7_mul(const double A[7][7], const double B[7][7], double C[7][7]) {
//    for (int i=0;i<7;i++) {
//        for (int j=0;j<7;j++) {
//            double s = 0.0;
//            for (int k=0;k<7;k++) s += A[i][k]*B[k][j];
//            C[i][j] = s;
//        }
//    }
//}
//
//static void mat7_add(const double A[7][7], const double B[7][7], double C[7][7]) {
//    for (int i=0;i<7;i++)
//        for (int j=0;j<7;j++)
//            C[i][j] = A[i][j] + B[i][j];
//}
//
//static void mat7_sub(const double A[7][7], const double B[7][7], double C[7][7]) {
//    for (int i=0;i<7;i++)
//        for (int j=0;j<7;j++)
//            C[i][j] = A[i][j] - B[i][j];
//}
//
//static void mat7x3_mul(const double A[7][7], const double B[7][3], double C[7][3]) {
//    for (int i=0;i<7;i++) {
//        for (int j=0;j<3;j++) {
//            double s=0.0;
//            for (int k=0;k<7;k++) s += A[i][k]*B[k][j];
//            C[i][j] = s;
//        }
//    }
//}
//
//static void mat3_mul(const double A[3][3], const double B[3][3], double C[3][3]) {
//    for (int i=0;i<3;i++) {
//        for (int j=0;j<3;j++) {
//            double s=0.0;
//            for (int k=0;k<3;k++) s += A[i][k]*B[k][j];
//            C[i][j] = s;
//        }
//    }
//}
//
//static void mat3_add(const double A[3][3], const double B[3][3], double C[3][3]) {
//    for (int i=0;i<3;i++)
//        for (int j=0;j<3;j++)
//            C[i][j] = A[i][j] + B[i][j];
//}
//
//static void mat3_inv(const double A[3][3], double Ainv[3][3]) {
//    // Inverse of 3x3 via adjugate/det. Assumes invertible.
//    double a=A[0][0], b=A[0][1], c=A[0][2];
//    double d=A[1][0], e=A[1][1], f=A[1][2];
//    double g=A[2][0], h=A[2][1], i=A[2][2];
//
//    double A11 = (e*i - f*h);
//    double A12 = -(d*i - f*g);
//    double A13 = (d*h - e*g);
//
//    double A21 = -(b*i - c*h);
//    double A22 = (a*i - c*g);
//    double A23 = -(a*h - b*g);
//
//    double A31 = (b*f - c*e);
//    double A32 = -(a*f - c*d);
//    double A33 = (a*e - b*d);
//
//    double det = a*A11 + b*A12 + c*A13;
//
//    // If you want safety: check |det| > tiny.
//    double invdet = 1.0 / det;
//
//    Ainv[0][0] = A11 * invdet;
//    Ainv[0][1] = A21 * invdet;
//    Ainv[0][2] = A31 * invdet;
//
//    Ainv[1][0] = A12 * invdet;
//    Ainv[1][1] = A22 * invdet;
//    Ainv[1][2] = A32 * invdet;
//
//    Ainv[2][0] = A13 * invdet;
//    Ainv[2][1] = A23 * invdet;
//    Ainv[2][2] = A33 * invdet;
//}
//
//static void mat3_vec3_mul(const double A[3][3], const double v[3], double out[3]) {
//    for (int i=0;i<3;i++) {
//        out[i] = A[i][0]*v[0] + A[i][1]*v[1] + A[i][2]*v[2];
//    }
//}
//
//static void mat7x3_mul_3x3(const double A[7][3], const double B[3][3], double C[7][3]) {
//    for (int i=0;i<7;i++) {
//        for (int j=0;j<3;j++) {
//            double s=0.0;
//            for (int k=0;k<3;k++) s += A[i][k]*B[k][j];
//            C[i][j] = s;
//        }
//    }
//}
//
//static void mat7x3_mul_3vec(const double A[7][3], const double v[3], double out[7]) {
//    for (int i=0;i<7;i++) {
//        out[i] = A[i][0]*v[0] + A[i][1]*v[1] + A[i][2]*v[2];
//    }
//}
//
//static void mat3x7_mul_7x7(const double A[3][7], const double B[7][7], double C[3][7]) {
//    for (int i=0;i<3;i++) {
//        for (int j=0;j<7;j++) {
//            double s=0.0;
//            for (int k=0;k<7;k++) s += A[i][k]*B[k][j];
//            C[i][j] = s;
//        }
//    }
//}
//
//static void mat3x7_mul_7x3(const double A[3][7], const double B[7][3], double C[3][3]) {
//    for (int i=0;i<3;i++) {
//        for (int j=0;j<3;j++) {
//            double s=0.0;
//            for (int k=0;k<7;k++) s += A[i][k]*B[k][j];
//            C[i][j] = s;
//        }
//    }
//}
//
//// ------------------------- EKF core: F, W, H, f, h -------------------------
//
//static void get_F(const double x[7], const double w[3], double dt, double F[7][7]) {
//    const double qw=x[0], qx=x[1], qy=x[2], qz=x[3];
//    const double bx=x[4], by=x[5], bz=x[6];
//    const double wx=w[0], wy=w[1], wz=w[2];
//
//    // zero then fill
//    memset(F, 0, sizeof(double)*7*7);
//    F[0][0]=1; F[0][1]=dt*(-wx + bx)/2; F[0][2]=dt*(-wy + by)/2; F[0][3]=dt*(-wz + bz)/2; F[0][4]= dt*qx/2; F[0][5]= dt*qy/2; F[0][6]= dt*qz/2;
//    F[1][0]=dt*(wx - bx)/2; F[1][1]=1; F[1][2]=dt*( wz - bz)/2; F[1][3]=dt*(-wy + by)/2; F[1][4]=-dt*qw/2; F[1][5]= dt*qz/2; F[1][6]=-dt*qy/2;
//    F[2][0]=dt*(wy - by)/2; F[2][1]=dt*(-wz + bz)/2; F[2][2]=1; F[2][3]=dt*( wx - bx)/2; F[2][4]=-dt*qz/2; F[2][5]=-dt*qw/2; F[2][6]= dt*qx/2;
//    F[3][0]=dt*(wz - bz)/2; F[3][1]=dt*( wy - by)/2; F[3][2]=dt*(-wx + bx)/2; F[3][3]=1; F[3][4]= dt*qy/2; F[3][5]=-dt*qx/2; F[3][6]=-dt*qw/2;
//
//    F[4][4]=1; F[5][5]=1; F[6][6]=1;
//}
//
//static void get_W(const double x[7], double dt, double W[7][3]) {
//    const double qw=x[0], qx=x[1], qy=x[2], qz=x[3];
//    double s = dt / 2.0;
//
//    W[0][0] = s*(-qx); W[0][1] = s*(-qy); W[0][2] = s*(-qz);
//    W[1][0] = s*( qw); W[1][1] = s*(-qz); W[1][2] = s*( qy);
//    W[2][0] = s*( qz); W[2][1] = s*( qw); W[2][2] = s*(-qx);
//    W[3][0] = s*(-qy); W[3][1] = s*( qx); W[3][2] = s*( qw);
//
//    W[4][0] = 0; W[4][1] = 0; W[4][2] = 0;
//    W[5][0] = 0; W[5][1] = 0; W[5][2] = 0;
//    W[6][0] = 0; W[6][1] = 0; W[6][2] = 0;
//}
//
//static void get_H(const double x[7], double H[3][7]) {
//    const double qw=x[0], qx=x[1], qy=x[2], qz=x[3];
//    double s = 2.0 * G;
//
//    // 3x7
//    H[0][0]= s*( qy); H[0][1]= s*(-qz); H[0][2]= s*( qw); H[0][3]= s*(-qx); H[0][4]=0; H[0][5]=0; H[0][6]=0;
//    H[1][0]= s*(-qx); H[1][1]= s*(-qw); H[1][2]= s*(-qz); H[1][3]= s*(-qy); H[1][4]=0; H[1][5]=0; H[1][6]=0;
//    H[2][0]= s*(-qw); H[2][1]= s*( qx); H[2][2]= s*( qy); H[2][3]= s*(-qz); H[2][4]=0; H[2][5]=0; H[2][6]=0;
//}
//
//static void f_state(const double x[7], const double w[3], double dt, double x_next[7]) {
//    // x = [q(4), b(3)]
//    double q[4] = { x[0], x[1], x[2], x[3] };
//    double b[3] = { x[4], x[5], x[6] };
//
//    double d_ang[3] = { (w[0]-b[0])*dt, (w[1]-b[1])*dt, (w[2]-b[2])*dt };
//    double dq[4];
//    quat_from_rotation_vector(d_ang, 0.0, dq);
//
//    double q_new[4];
//    quat_multiply(q, dq, q_new);
//    quat_normalize(q_new);
//
//    x_next[0]=q_new[0]; x_next[1]=q_new[1]; x_next[2]=q_new[2]; x_next[3]=q_new[3];
//    x_next[4]=b[0];     x_next[5]=b[1];     x_next[6]=b[2];
//}
//
//static void h_meas(const double x[7], double out[3]) {
//    // h(x) = R(q)^T * [0,0,-g]
//    double q[4] = { x[0], x[1], x[2], x[3] };
//    double R[3][3], RT[3][3];
//    quat_to_matrix(q, R);
//    mat3_transpose(R, RT);
//
//    const double v[3] = { 0.0, 0.0, -G };
//    mat3_vec3_mul(RT, v, out);
//}
//
//// ------------------------- EKF struct + API -------------------------
//
//
//
//void ekf_init(EKF *ekf,
//              const double q0[4], const double b0[3],
//              double init_gyro_bias_err,
//              double gyro_noise,
//              double gyro_bias_noise,
//              double accelerometer_noise)
//{
//    // state
//    ekf->x[0]=q0[0]; ekf->x[1]=q0[1]; ekf->x[2]=q0[2]; ekf->x[3]=q0[3];
//    ekf->x[4]=b0[0]; ekf->x[5]=b0[1]; ekf->x[6]=b0[2];
//
//    // P
//    memset(ekf->P, 0, sizeof(double)*7*7);
//    for (int i=0;i<4;i++) ekf->P[i][i] = 0.01;
//    double bvar = init_gyro_bias_err * init_gyro_bias_err;
//    for (int i=4;i<7;i++) ekf->P[i][i] = bvar;
//
//    // Q (gyro)
//    memset(ekf->Q, 0, sizeof(double)*3*3);
//    double gvar = gyro_noise * gyro_noise;
//    for (int i=0;i<3;i++) ekf->Q[i][i] = gvar;
//
//    // Q_bias
//    memset(ekf->Q_bias, 0, sizeof(double)*7*7);
//    double bgvar = gyro_bias_noise * gyro_bias_noise;
//    for (int i=4;i<7;i++) ekf->Q_bias[i][i] = bgvar;
//
//    // R (accel)
//    memset(ekf->R, 0, sizeof(double)*3*3);
//    double avar = accelerometer_noise * accelerometer_noise;
//    for (int i=0;i<3;i++) ekf->R[i][i] = avar;
//
//    // ensure quaternion is unit
//    {
//        double q[4] = { ekf->x[0], ekf->x[1], ekf->x[2], ekf->x[3] };
//        quat_normalize(q);
//        ekf->x[0]=q[0]; ekf->x[1]=q[1]; ekf->x[2]=q[2]; ekf->x[3]=q[3];
//    }
//}
//
//void ekf_predict(EKF *ekf, const double w_meas[3], double dt) {
//    double F[7][7];
//    double W[7][3];
//    get_F(ekf->x, w_meas, dt, F);
//    get_W(ekf->x, dt, W);
//
//    // x = f(x,w,dt)
//    double x_next[7];
//    f_state(ekf->x, w_meas, dt, x_next);
//    memcpy(ekf->x, x_next, sizeof(double)*7);
//
//    // P = F P F^T + W Q W^T + Q_bias
//    double FP[7][7], FT[7][7], FPFt[7][7];
//    mat7_mul(F, ekf->P, FP);
//    mat7_transpose(F, FT);
//    mat7_mul(FP, FT, FPFt);
//
//    // W Q W^T
//    double WQ[7][3];
//    mat7x3_mul_3x3(W, ekf->Q, WQ);
//
//    // (WQ) * W^T -> 7x7
//    double Wt[3][7];
//    for (int i=0;i<3;i++)
//        for (int j=0;j<7;j++)
//            Wt[i][j] = W[j][i];
//
//    double WQWt[7][7];
//    for (int i=0;i<7;i++) {
//        for (int j=0;j<7;j++) {
//            double s=0.0;
//            for (int k=0;k<3;k++) s += WQ[i][k]*Wt[k][j];
//            WQWt[i][j] = s;
//        }
//    }
//
//    double tmp[7][7];
//    mat7_add(FPFt, WQWt, tmp);
//    mat7_add(tmp, ekf->Q_bias, ekf->P);
//}
//
//void ekf_update(EKF *ekf, const double a_meas[3]) {
//    // z = g * normalize(a)
//    double a_unit[3];
//    vec3_normalize(a_meas, a_unit);
//    double z[3] = { G*a_unit[0], G*a_unit[1], G*a_unit[2] };
//
//    // y = z - h(x)
//    double hx[3];
//    h_meas(ekf->x, hx);
//    double y[3] = { z[0]-hx[0], z[1]-hx[1], z[2]-hx[2] };
//
//    // H, S = H P H^T + R
//    double H[3][7];
//    get_H(ekf->x, H);
//
//    double HP[3][7];
//    mat3x7_mul_7x7(H, ekf->P, HP);
//
//    double HT[7][3];
//    for (int i=0;i<7;i++)
//        for (int j=0;j<3;j++)
//            HT[i][j] = H[j][i];
//
//    double HPHt[3][3];
//    mat3x7_mul_7x3(HP, HT, HPHt);
//
//    double S[3][3];
//    mat3_add(HPHt, ekf->R, S);
//
//    // K = P H^T S^-1
//    double Sinv[3][3];
//    mat3_inv(S, Sinv);
//
//    double PHT[7][3];
//    mat7x3_mul(ekf->P, HT, PHT);
//
//    double K[7][3];
//    mat7x3_mul_3x3(PHT, Sinv, K);
//
//    // x = x + K y
//    double Ky[7];
//    mat7x3_mul_3vec(K, y, Ky);
//    for (int i=0;i<7;i++) ekf->x[i] += Ky[i];
//
//    // normalize quaternion part
//    {
//        double q[4] = { ekf->x[0], ekf->x[1], ekf->x[2], ekf->x[3] };
//        quat_normalize(q);
//        ekf->x[0]=q[0]; ekf->x[1]=q[1]; ekf->x[2]=q[2]; ekf->x[3]=q[3];
//    }
//
//    // P = (I - K H) P
//    double KH[7][7];
//    // KH = K(7x3) * H(3x7)
//    for (int i=0;i<7;i++) {
//        for (int j=0;j<7;j++) {
//            double s=0.0;
//            for (int k=0;k<3;k++) s += K[i][k]*H[k][j];
//            KH[i][j] = s;
//        }
//    }
//
//    double I[7][7], IminusKH[7][7];
//    mat7_identity(I);
//    mat7_sub(I, KH, IminusKH);
//
//    double newP[7][7];
//    mat7_mul(IminusKH, ekf->P, newP);
//    memcpy(ekf->P, newP, sizeof(double)*7*7);
//}
//
//
//
//
//
//








#include "Kalman.h"
#include <math.h>
#include <stdbool.h>

static inline bool inv3x3(const float M[9], float invOut[9])
{
    // M is row-major: [ m00 m01 m02; m10 m11 m12; m20 m21 m22 ]
    const float m00 = M[0], m01 = M[1], m02 = M[2];
    const float m10 = M[3], m11 = M[4], m12 = M[5];
    const float m20 = M[6], m21 = M[7], m22 = M[8];

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
    if (fabsf(det) < 1e-9f) {
        return false;
    }

    const float invDet = 1.0f / det;

    // inv(M) = adj(M) / det = (Cofactor^T)/det
    invOut[0] = c00 * invDet;
    invOut[1] = c10 * invDet;
    invOut[2] = c20 * invDet;

    invOut[3] = c01 * invDet;
    invOut[4] = c11 * invDet;
    invOut[5] = c21 * invDet;

    invOut[6] = c02 * invDet;
    invOut[7] = c12 * invDet;
    invOut[8] = c22 * invDet;

    return true;
}

void Kalman_Init(Kalman *kal, float Pinit, float *Q, float *R)
{
    kal->phi_rad   = 0.0f;
    kal->theta_rad = 0.0f;

    // P stored row-major: [p00 p01; p10 p11]
    kal->P[0] = Pinit;  kal->P[1] = 0.0f;
    kal->P[2] = 0.0f;   kal->P[3] = Pinit;

    kal->Q[0] = Q[0];
    kal->Q[1] = Q[1];

    kal->R[0] = R[0];
    kal->R[1] = R[1];
    kal->R[2] = R[2];
}

void Kalman_Predict(Kalman *kal, float *gyr_rdps, float T)
{
    const float p = gyr_rdps[0];
    const float q = gyr_rdps[1];
    const float r = gyr_rdps[2];

    // Cache angles
    const float phi   = kal->phi_rad;
    const float theta = kal->theta_rad;

    // Trig (use float versions!)
    const float sp = sinf(phi);
    const float cp = cosf(phi);
    const float tt = tanf(theta);

    // ===== Predict state: x_{k+1} = x_k + T*f(x,u)
    const float dphi   = p + tt * (q * sp + r * cp);
    const float dtheta =       (q * cp - r * sp);

    kal->phi_rad   = phi   + T * dphi;
    kal->theta_rad = theta + T * dtheta;

    // Recompute trig at predicted state
    const float sp2 = sinf(kal->phi_rad);
    const float cp2 = cosf(kal->phi_rad);
    const float st2 = sinf(kal->theta_rad);
    const float ct2 = cosf(kal->theta_rad);

    // Avoid division by 0 near +-90deg pitch
    const float inv_ct = (fabsf(ct2) > 1e-6f) ? (1.0f / ct2) : (1.0f / (ct2 >= 0 ? 1e-6f : -1e-6f));
    const float tt2    = st2 * inv_ct;
    const float sec2   = 1.0f + tt2 * tt2; // sec^2(theta) = 1 + tan^2(theta)

    const float u1 = (q * sp2 + r * cp2);   // q*sin(phi) + r*cos(phi)
    const float u2 = (q * cp2 - r * sp2);   // q*cos(phi) - r*sin(phi)

    // ===== Jacobian A = df/dx  (2x2, row-major [a00 a01; a10 a11])
    const float a00 = tt2 * u2;
    const float a01 = u1 * sec2;
    const float a10 = -u1;
    const float a11 = 0.0f;

    // ===== Pdot = A*P + P*A' + Q  (Q diagonal: [Q0, Q1])
    const float p00 = kal->P[0], p01 = kal->P[1];
    const float p10 = kal->P[2], p11 = kal->P[3];

    const float dP00 = kal->Q[0] + 2.0f*a00*p00 + a01*p01 + a01*p10;
    const float dP01 =              a00*p01 + a10*p00 + a01*p11 + a11*p01;
    const float dP10 =              a00*p10 + a10*p00 + a01*p11 + a11*p10;
    const float dP11 = kal->Q[1] +  a10*p01 + a10*p10 + 2.0f*a11*p11;

    kal->P[0] = p00 + T * dP00;
    kal->P[1] = p01 + T * dP01;
    kal->P[2] = p10 + T * dP10;
    kal->P[3] = p11 + T * dP11;

    // Enforce symmetry (numerical hygiene)
    const float off = 0.5f * (kal->P[1] + kal->P[2]);
    kal->P[1] = off;
    kal->P[2] = off;
}

void Kalman_Update(Kalman *kal, float *acc_mps2)
{
    const float ax = acc_mps2[0];
    const float ay = acc_mps2[1];
    const float az = acc_mps2[2];

    // ===== Normalize accel (use direction only)
    const float an = sqrtf(ax*ax + ay*ay + az*az);
    if (an < 1e-6f) {
        return;
    }

    // Optional: gate accel update when not close to 1g (boost/high-g -> bad gravity ref)
    // Tune threshold as needed (0.3g..0.6g typical)
    const float gate = 0.1f * g;
    if (fabsf(an - g) > gate) {
        return;
    }

    const float y0 = ax / an;
    const float y1 = ay / an;
    const float y2 = az / an;

    // ===== Trig
    const float phi   = kal->phi_rad;
    const float theta = kal->theta_rad;

    const float sp = sinf(phi);
    const float cp = cosf(phi);
    const float st = sinf(theta);
    const float ct = cosf(theta);

    // ===== Measurement model h(x) (unit gravity direction in body frame)
    const float h0 =  st;
    const float h1 = -ct * sp;
    const float h2 = -ct * cp;

    // ===== C = dh/dx (3x2)
    // [ 0,      ct
    //  -cp*ct,  sp*st
    //   sp*ct,  cp*st ]
    const float c00 = 0.0f;
    const float c01 = ct;

    const float c10 = -cp * ct;
    const float c11 =  sp * st;

    const float c20 =  sp * ct;
    const float c21 =  cp * st;

    // ===== Innovation v = y - h
    const float v0 = y0 - h0;
    const float v1 = y1 - h1;
    const float v2 = y2 - h2;

    // ===== Read P
    const float p00 = kal->P[0], p01 = kal->P[1];
    const float p10 = kal->P[2], p11 = kal->P[3];

    // ===== PCt = P * C'  (2x3)
    const float pc00 = p00*c00 + p01*c01;
    const float pc01 = p00*c10 + p01*c11;
    const float pc02 = p00*c20 + p01*c21;

    const float pc10 = p10*c00 + p11*c01;
    const float pc11 = p10*c10 + p11*c11;
    const float pc12 = p10*c20 + p11*c21;

    // ===== S = C*PCt + R  (3x3), R diagonal
    const float r0 = kal->R[0];
    const float r1 = kal->R[1];
    const float r2 = kal->R[2];

    float S[9];
    S[0] = c00*pc00 + c01*pc10 + r0;
    S[1] = c00*pc01 + c01*pc11;
    S[2] = c00*pc02 + c01*pc12;

    S[3] = c10*pc00 + c11*pc10;
    S[4] = c10*pc01 + c11*pc11 + r1;
    S[5] = c10*pc02 + c11*pc12;

    S[6] = c20*pc00 + c21*pc10;
    S[7] = c20*pc01 + c21*pc11;
    S[8] = c20*pc02 + c21*pc12 + r2;

    float invS[9];
    if (!inv3x3(S, invS)) {
        return;
    }

    // ===== K = PCt * invS  (2x3)
    const float k00 = pc00*invS[0] + pc01*invS[3] + pc02*invS[6];
    const float k01 = pc00*invS[1] + pc01*invS[4] + pc02*invS[7];
    const float k02 = pc00*invS[2] + pc01*invS[5] + pc02*invS[8];

    const float k10 = pc10*invS[0] + pc11*invS[3] + pc12*invS[6];
    const float k11 = pc10*invS[1] + pc11*invS[4] + pc12*invS[7];
    const float k12 = pc10*invS[2] + pc11*invS[5] + pc12*invS[8];

    // ===== State update: x = x + K*v
    kal->phi_rad   += (k00*v0 + k01*v1 + k02*v2);
    kal->theta_rad += (k10*v0 + k11*v1 + k12*v2);

    // ===== Covariance update (Joseph form, R diagonal)
    // KC = K*C  (2x2)
    const float KC00 = k00*c00 + k01*c10 + k02*c20;
    const float KC01 = k00*c01 + k01*c11 + k02*c21;
    const float KC10 = k10*c00 + k11*c10 + k12*c20;
    const float KC11 = k10*c01 + k11*c11 + k12*c21;

    // M = I - KC
    const float M00 = 1.0f - KC00;
    const float M01 =      - KC01;
    const float M10 =      - KC10;
    const float M11 = 1.0f - KC11;

    // Pm = M*P*M'
    const float t00 = M00*p00 + M01*p10;
    const float t01 = M00*p01 + M01*p11;
    const float t10 = M10*p00 + M11*p10;
    const float t11 = M10*p01 + M11*p11;

    float Pm00 = t00*M00 + t01*M01;
    float Pm01 = t00*M10 + t01*M11;
    float Pm10 = t10*M00 + t11*M01;
    float Pm11 = t10*M10 + t11*M11;

    // KRKt = K*R*K' (R diagonal)
    const float KRKt00 = r0*k00*k00 + r1*k01*k01 + r2*k02*k02;
    const float KRKt01 = r0*k00*k10 + r1*k01*k11 + r2*k02*k12;
    const float KRKt11 = r0*k10*k10 + r1*k11*k11 + r2*k12*k12;

    kal->P[0] = Pm00 + KRKt00;
    kal->P[1] = Pm01 + KRKt01;
    kal->P[2] = Pm10 + KRKt01;   // symmetric counterpart
    kal->P[3] = Pm11 + KRKt11;

    // Enforce symmetry
    const float off = 0.5f * (kal->P[1] + kal->P[2]);
    kal->P[1] = off;
    kal->P[2] = off;
}
