#include "madgwick.h"

struct quaternion q_est = { 1, 0, 0, 0 }; // initialize with as unit vector with real component = 1

struct quaternion quat_mult(struct quaternion L, struct quaternion R) {


    struct quaternion product;
    product.q1 = (L.q1 * R.q1) - (L.q2 * R.q2) - (L.q3 * R.q3) - (L.q4 * R.q4);
    product.q2 = (L.q1 * R.q2) + (L.q2 * R.q1) + (L.q3 * R.q4) - (L.q4 * R.q3);
    product.q3 = (L.q1 * R.q3) - (L.q2 * R.q4) + (L.q3 * R.q1) + (L.q4 * R.q2);
    product.q4 = (L.q1 * R.q4) + (L.q2 * R.q3) - (L.q3 * R.q2) + (L.q4 * R.q1);

    return product;
}

// --- MODIFIED: Function signature updated for 9-DoF and variable dt ---
void imu_filter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float dt) {

    //Variables and constants
    struct quaternion q_est_prev = q_est;
    struct quaternion q_est_dot = { 0 };    // used as a place holder in equations 42 and 43
    struct quaternion q_a = { 0, ax, ay, az }; // equation (24) raw acceleration values
    struct quaternion q_m = { 0, mx, my, mz }; // --- NEW: raw magnetometer values ---

    float F_g[3] = { 0 }; // equation(15/21/25) objective function for gravity
    float J_g[3][4] = { 0 }; // jacobian matrix for gravity
    
    // --- NEW: Variables for magnetometer ---
    float F_m[3] = { 0 };     // objective function for magnetism
    float J_m[3][4] = { 0 };  // jacobian matrix for magnetism
    struct quaternion gradient_g = { 0 }; // gravity gradient
    struct quaternion gradient_m = { 0 }; // magnetism gradient
    struct quaternion gradient_total = { 0 }; // combined gradient
    struct quaternion h = { 0 };    // estimated magnetic field vector in earth frame
    float b_x = 0.0f;             // earth's magnetic field horizontal component
    float b_z = 0.0f;             // earth's magnetic field vertical component

    /* Integrate angluar velocity to obtain position in angles. */
    struct quaternion q_w; // equation (10), places gyroscope readings in a quaternion
    q_w.q1 = 0; // the real component is zero
    q_w.q2 = gx;
    q_w.q3 = gy;
    q_w.q4 = gz;

    quat_scalar(&q_w, 0.5); // equation (12) dq/dt = (1/2)q*w
    q_w = quat_mult(q_est_prev, q_w); // equation (12)

    // --- Normalize sensor measurements ---
    quat_Normalization(&q_a); // normalize the acceleration quaternion
    quat_Normalization(&q_m); // --- NEW: normalize the magnetometer quaternion ---

    // --- Pre-calculate reused quaternion components ---
    float q1 = q_est_prev.q1;
    float q2 = q_est_prev.q2;
    float q3 = q_est_prev.q3;
    float q4 = q_est_prev.q4;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _q1q1 = q1 * q1;
    float _q2q2 = q2 * q2;
    float _q3q3 = q3 * q3;
    float _q4q4 = q4 * q4;
    
    // --- 1. GRAVITY GRADIENT DESCENT ---

    //Compute the objective function for gravity, equation(25)
    F_g[0] = 2.0f * (q2 * q4 - q1 * q3) - q_a.q2;
    F_g[1] = 2.0f * (q1 * q2 + q3 * q4) - q_a.q3;
    F_g[2] = 2.0f * (0.5f - _q2q2 - _q3q3) - q_a.q4;

    //Compute the Jacobian matrix, equation (26), for gravity
    J_g[0][0] = -_2q3;
    J_g[0][1] = _2q4;
    J_g[0][2] = -_2q1;
    J_g[0][3] = _2q2;

    J_g[1][0] = _2q2;
    J_g[1][1] = _2q1;
    J_g[1][2] = _2q4;
    J_g[1][3] = _2q3;

    J_g[2][0] = 0;
    J_g[2][1] = -4.0f * q2;
    J_g[2][2] = -4.0f * q3;
    J_g[2][3] = 0;

    // now computer the gradient, equation (20), gradient_g = J_g'*F_g
    gradient_g.q1 = J_g[0][0] * F_g[0] + J_g[1][0] * F_g[1] + J_g[2][0] * F_g[2];
    gradient_g.q2 = J_g[0][1] * F_g[0] + J_g[1][1] * F_g[1] + J_g[2][1] * F_g[2];
    gradient_g.q3 = J_g[0][2] * F_g[0] + J_g[1][2] * F_g[1] + J_g[2][2] * F_g[2];
    gradient_g.q4 = J_g[0][3] * F_g[0] + J_g[1][3] * F_g[1] + J_g[2][3] * F_g[2];


    // --- 2. MAGNETOMETER GRADIENT DESCENT ---
    
    // Calculate reference magnetic field vector (h) in Earth frame (Eq 38)
    // h = q_est * q_m * q_est_conjugate
    struct quaternion q_est_conj = quat_conjugate(q_est_prev);
    h = quat_mult(q_est_prev, q_m);
    h = quat_mult(h, q_est_conj);

    // Calculate reference magnetic field direction (b) (Eq 39)
    // b_x = sqrt(h_x^2 + h_y^2)
    // b_z = h_z
    b_x = sqrtf(h.q2 * h.q2 + h.q3 * h.q3);
    b_z = h.q4;

    // Pre-calculate reused mag terms
    float _2bx = 2.0f * b_x;
    float _2bz = 2.0f * b_z;
    float _4bx = 4.0f * b_x;
    float _4bz = 4.0f * b_z;

    //Compute the objective function for magnetism, equation(40)
    F_m[0] = _2bx * (0.5f - _q3q3 - _q4q4) + _2bz * (q2 * q4 - q1 * q3) - q_m.q2;
    F_m[1] = _2bx * (q2 * q3 - q1 * q4) + _2bz * (q1 * q2 + q3 * q4) - q_m.q3;
    F_m[2] = _2bx * (q1 * q3 + q2 * q4) + _2bz * (0.5f - _q2q2 - _q3q3) - q_m.q4;

    //Compute the Jacobian matrix, equation (41), for magnetism
    J_m[0][0] = -_2bz * q3;
    J_m[0][1] = _2bz * q4;
    J_m[0][2] = -_4bx * q3 + _2bz * q2;
    J_m[0][3] = -_4bx * q4 + _2bz * q1;

    J_m[1][0] = -_2bx * q4 + _2bz * q2;
    J_m[1][1] = _2bx * q3 + _2bz * q1;
    J_m[1][2] = _2bx * q2 + _2bz * q4;
    J_m[1][3] = -_2bx * q1 + _2bz * q3;

    J_m[2][0] = _2bx * q3;
    J_m[2][1] = _2bx * q4 - _4bz * q2;
    J_m[2][2] = _2bx * q1 - _4bz * q3;
    J_m[2][3] = _2bx * q2;

    // now computer the magnetometer gradient, gradient_m = J_m'*F_m
    gradient_m.q1 = J_m[0][0] * F_m[0] + J_m[1][0] * F_m[1] + J_m[2][0] * F_m[2];
    gradient_m.q2 = J_m[0][1] * F_m[0] + J_m[1][1] * F_m[1] + J_m[2][1] * F_m[2];
    gradient_m.q3 = J_m[0][2] * F_m[0] + J_m[1][2] * F_m[1] + J_m[2][2] * F_m[2];
    gradient_m.q4 = J_m[0][3] * F_m[0] + J_m[1][3] * F_m[1] + J_m[2][3] * F_m[2];
    

    // --- 3. COMBINE AND INTEGRATE ---
    
    // Combine gradients
    quat_add(&gradient_total, gradient_g, gradient_m);
    
    // --- MODIFIED: Use combined gradient ---
    quat_scalar(&gradient_total, BETA); // multiply total gradient by beta
    quat_sub(&q_est_dot, q_w, gradient_total); // subtract above from q_w, the integrated gyro quaternion

    // --- MODIFIED: Use variable dt ---
    quat_scalar(&q_est_dot, dt);
    quat_add(&q_est, q_est_prev, q_est_dot); // Integrate orientation rate to find position
    quat_Normalization(&q_est); // normalize the orientation of the estimate
}

void eulerAngles(struct quaternion q, float* roll, float* pitch, float* yaw) {

    *yaw = atan2f((2.0f * q.q2 * q.q3 - 2.0f * q.q1 * q.q4), (2.0f * q.q1 * q.q1 + 2.0f * q.q2 * q.q2 - 1.0f)); // equation (7)
    *pitch = -asinf(2.0f * q.q2 * q.q4 + 2.0f * q.q1 * q.q3); // equatino (8)
    *roll = atan2f((2.0f * q.q3 * q.q4 - 2.0f * q.q1 * q.q2), (2.0f * q.q1 * q.q1 + 2.0f * q.q4 * q.q4 - 1.0f));

    *yaw *= (180.0f / PI);
    *pitch *= (180.0f / PI);
    *roll *= (180.0f / PI);

}
