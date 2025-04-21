#ifndef MADGWICK_AHRS_H
#define MADGWICK_AHRS_H

class Madgwick {
private:
    float beta;             // algorithm gain
    float q0, q1, q2, q3;  // quaternion of sensor frame relative to auxiliary frame
    
public:
    // Constructor
    Madgwick(float beta = 0.1f);
    
    // Update IMU (gyroscope + accelerometer)
    void updateIMU(float gx, float gy, float gz, float ax, float ay, float az, float dt);
    
    // Update with magnetometer (9DOF)
    void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float dt);
    
    // Get Euler angles in radians
    void getEulerAngles(float& roll, float& pitch, float& yaw);
    
    // Get quaternion components
    void getQuaternion(float& q0, float& q1, float& q2, float& q3);
    
    // Get gravity vector
    void getGravityVector(float& gx, float& gy, float& gz);
    
    // Reset orientation
    void reset();
};

#endif // MADGWICK_AHRS_H