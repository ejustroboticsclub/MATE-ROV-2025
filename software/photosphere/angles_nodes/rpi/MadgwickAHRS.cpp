#include "MadgwickAHRS.hpp"
#include <cmath>
#include <algorithm>

Madgwick::Madgwick(float beta) : beta(beta), q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f) {}

void Madgwick::updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Normalize accelerometer measurement
    float norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return; // handle NaN
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Compute objective function and Jacobian
    float f1 = 2.0f * (q1 * q3 - q0 * q2) - ax;
    float f2 = 2.0f * (q0 * q1 + q2 * q3) - ay;
    float f3 = 2.0f * (0.5f - q1 * q1 - q2 * q2) - az;

    // Compute Jacobian
    float J_11or24 = 2.0f * q2;
    float J_12or23 = 2.0f * q3;
    float J_13or22 = 2.0f * q0;
    float J_14or21 = 2.0f * q1;
    float J_32 = 2.0f * J_14or21;
    float J_33 = 2.0f * J_11or24;

    // Compute gradient
    float hatDot1 = J_14or21 * f2 - J_11or24 * f1;
    float hatDot2 = J_12or23 * f1 + J_13or22 * f2 - J_32 * f3;
    float hatDot3 = J_12or23 * f2 - J_33 * f3 - J_13or22 * f1;
    float hatDot4 = J_14or21 * f1 + J_11or24 * f2;

    // Normalize gradient
    norm = sqrt(hatDot1 * hatDot1 + hatDot2 * hatDot2 + hatDot3 * hatDot3 + hatDot4 * hatDot4);
    norm = 1.0f / norm;
    hatDot1 *= norm;
    hatDot2 *= norm;
    hatDot3 *= norm;
    hatDot4 *= norm;

    // Compute estimated gyroscope biases
    float gerrx = gx - beta * hatDot1;
    float gerry = gy - beta * hatDot2;
    float gerrz = gz - beta * hatDot3;

    // Compute quaternion derivative
    float qDot1 = 0.5f * (-q1 * gerrx - q2 * gerry - q3 * gerrz);
    float qDot2 = 0.5f * (q0 * gerrx + q2 * gerrz - q3 * gerry);
    float qDot3 = 0.5f * (q0 * gerry - q1 * gerrz + q3 * gerrx);
    float qDot4 = 0.5f * (q0 * gerrz + q1 * gerry - q2 * gerrx);

    // Integrate to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalize quaternion
    norm = sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    norm = 1.0f / norm;
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
}

void Madgwick::update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt) {
    // Convert gyroscope degrees/sec to radians/sec
    gx *= 0.0174533f;
    gy *= 0.0174533f;
    gz *= 0.0174533f;

    // Normalize accelerometer measurement
    float norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f) return;
    norm = 1.0f / norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalize magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f) return;
    norm = 1.0f / norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    float hx = 2.0f * mx * (0.5f - q2 * q2 - q3 * q3) + 2.0f * my * (q1 * q2 - q0 * q3) + 2.0f * mz * (q1 * q3 + q0 * q2);
    float hy = 2.0f * mx * (q1 * q2 + q0 * q3) + 2.0f * my * (0.5f - q1 * q1 - q3 * q3) + 2.0f * mz * (q2 * q3 - q0 * q1);
    float bx = sqrt(hx * hx + hy * hy);
    float bz = 2.0f * mx * (q1 * q3 - q0 * q2) + 2.0f * my * (q2 * q3 + q0 * q1) + 2.0f * mz * (0.5f - q1 * q1 - q2 * q2);

    // Gradient descent algorithm corrective step
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    // Compute objective function and Jacobian
    s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    // Normalize step magnitude
    norm = 1.0f / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
    s0 *= norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz) - beta * s0;
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy) - beta * s1;
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx) - beta * s2;
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx) - beta * s3;

    // Integrate to yield quaternion
    q0 += qDot1 * dt;
    q1 += qDot2 * dt;
    q2 += qDot3 * dt;
    q3 += qDot4 * dt;

    // Normalize quaternion
    norm = 1.0f / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= norm;
    q1 *= norm;
    q2 *= norm;
    q3 *= norm;
}

void Madgwick::getEulerAngles(float& roll, float& pitch, float& yaw) {
    roll = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
    pitch = asinf(-2.0f * (q1*q3 - q0*q2));
    yaw = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);
}

void Madgwick::getQuaternion(float& w, float& x, float& y, float& z) {
    w = q0;
    x = q1;
    y = q2;
    z = q3;
}

void Madgwick::getGravityVector(float& gx, float& gy, float& gz) {
    gx = 2.0f * (q1*q3 - q0*q2);
    gy = 2.0f * (q0*q1 + q2*q3);
    gz = q0*q0 - q1*q1 - q2*q2 + q3*q3;
}

void Madgwick::reset() {
    q0 = 1.0f;
    q1 = 0.0f;
    q2 = 0.0f;
    q3 = 0.0f;
}