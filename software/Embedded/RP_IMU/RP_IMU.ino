#include <Arduino_LSM6DSOX.h>
#include <chrono>
#include <cmath>
#include <Wire.h>
#include "MS5837.h"

#define I2C 0x08

MS5837 sensor;

struct Quaternion {
    float w;
    float x;
    float y;
    float z;
};

struct EulerAngles {
  float roll;
  float pitch;
  float yaw;
};

// IMU readings
float Ax, Ay, Az;
float Gx, Gy, Gz;

// Quaternion variables
double q0_ = 1.0, q1_ = 0.0, q2_ = 0.0, q3_ = 0.0;
const double beta_ = 0.031; // Filter parameter
double dt_ = 0.0;         // Time delta for quaternion calculation

// Variables to store time points
unsigned long time_now_;
unsigned long time_last_;

bool use_filter_ = true; // Set this to true if you want to use accelerometer in the correction step

const double DEG_TO_RAD_ = M_PI / 180.0;  // Conversion factor from degrees to radians

float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;
float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;

// Number of readings for calibration
const int calibrationSamples = 2000;
bool isCalibrated = false;

int receivedValue;

void calibrateSensor() {
  float sumAx = 0, sumAy = 0, sumAz = 0;
  float sumGx = 0, sumGy = 0, sumGz = 0;

  // Read 1000 samples and accumulate them
  for (int i = 0; i < calibrationSamples; i++) {

    if (Serial.available()) {
      String command = Serial.readStringUntil('\n');

      // Check for the RESET command
      if (receivedValue == 0) {
        Serial.println("Reset command received. Resetting IMU...");

        // Reset all IMU-related variables and reinitialize IMU
        isCalibrated = false;
        q0_ = 1.0;
        q1_ = 0.0;
        q2_ = 0.0;
        q3_ = 0.0;
        accelOffsetX = 0;
        accelOffsetY = 0;
        accelOffsetZ = 0;
        gyroOffsetX = 0;
        gyroOffsetY = 0;
        gyroOffsetZ = 0;

        // Recalibrate the IMU
        //calibrateSensor();
        //time_last_ = micros();
        Serial.println("IMU reset complete.");
        return;
      }
    }


    // Temporary variables to store sensor data
    float Ax, Ay, Az;
    float Gx, Gy, Gz;

    // Read accelerometer and gyroscope values
    IMU.readAcceleration(Ax, Ay, Az);
    IMU.readGyroscope(Gx, Gy, Gz);

    // Accumulate the values
    sumAx += Ax * 9.80665;
    sumAy += Ay * 9.80665;
    sumAz += (Az * 9.80665 - 9.80665);

    sumGx += Gx * DEG_TO_RAD_;
    sumGy += Gy * DEG_TO_RAD_;
    sumGz += Gz * DEG_TO_RAD_;

    delay(10);  // Small delay to allow for sensor stability
    if ((i + 1) % 100 == 0 || i == calibrationSamples - 1) {
      Serial.println((String("Calibrating IMU " + String(i + 1) + " / " + calibrationSamples).c_str()));
    }
  }

  // Calculate the average values for calibration
  accelOffsetX = sumAx / calibrationSamples;
  accelOffsetY = sumAy / calibrationSamples;
  accelOffsetZ = sumAz / calibrationSamples;

  gyroOffsetX = sumGx / calibrationSamples;
  gyroOffsetY = sumGy / calibrationSamples;
  gyroOffsetZ = sumGz / calibrationSamples;

  isCalibrated = true;
  Serial.println("Calibration complete.");
}

EulerAngles quaternionToEulerDegrees(float w, float x, float y, float z) {
  EulerAngles angles;

  // Normalizing the quaternion to ensure it represents a valid rotation
  float norm = sqrt(w * w + x * x + y * y + z * z);
  w /= norm;
  x /= norm;
  y /= norm;
  z /= norm;

  // Roll (X-axis rotation)
  float sinr_cosp = 2.0f * (w * x + y * z);
  float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
  angles.roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / M_PI);

  // Pitch (Y-axis rotation)
  float sinp = 2.0f * (w * y - z * x);
  if (fabsf(sinp) >= 1.0f)
    angles.pitch = copysignf(90.0f, sinp); // Use 90 degrees if out of range
  else
    angles.pitch = asinf(sinp) * (180.0f / M_PI);

  // Yaw (Z-axis rotation)
  float siny_cosp = 2.0f * (w * z + x * y);
  float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
  angles.yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / M_PI);

  return angles;
}


void setup() {
 
  Wire.begin();
 
  Serial.begin(9600);

  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

 
  calibrateSensor();
  time_last_ = micros();


  while (!sensor.init()) {
    Serial.println("Init failed!");
    Serial.println("Are SDA/SCL connected correctly?");
    Serial.println("Blue Robotics Bar30: White=SDA, Green=SCL");
    Serial.println("\n\n\n");
    delay(5000);
  }
  
  sensor.setModel(MS5837::MS5837_02BA);
  sensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)


}

void loop() {
  // Normal operation if calibrated
  if (isCalibrated) {
    CalculationDeltaTime();

    if (IMU.accelerationAvailable() && IMU.gyroscopeAvailable()) {
      IMU.readAcceleration(Ax, Ay, Az);
      IMU.readGyroscope(Gx, Gy, Gz);

      double Gx_rad = Gx * DEG_TO_RAD_;
      double Gy_rad = Gy * DEG_TO_RAD_;
      double Gz_rad = Gz * DEG_TO_RAD_;

      CalculationQuaternion(
        Gx_rad - gyroOffsetX, Gy_rad - gyroOffsetY, Gz_rad - gyroOffsetZ,
        Ax * 9.80665 - accelOffsetX, Ay * 9.80665 - accelOffsetY, Az * 9.80665 - accelOffsetZ
      );
    }
    sensor.read();

    float depth = sensor.depth();
    
    byte buffer[32];
    float q0f = (float)q0_;
    float q1f = (float)q1_;
    float q2f = (float)q2_;
    float q3f = (float)q3_;

    memcpy(buffer, &q0f, 4);
    memcpy(buffer + 4, &q1f, 4);
    memcpy(buffer + 8, &q2f, 4);
    memcpy(buffer + 12, &q3f, 4);
    memcpy(buffer + 16, &Gx, 4);
    memcpy(buffer + 20, &Gy, 4);
    memcpy(buffer + 24, &Gz, 4);
    memcpy(buffer + 28, &depth, 4);

    // Send only 28 bytes
    Wire.beginTransmission(8);
    Wire.write(buffer, 32);
    Wire.endTransmission();

    Serial.print(q0_);Serial.print(",");
    Serial.print(q1_);Serial.print(",");
    Serial.print(q2_);Serial.print(",");
    Serial.print(q3_);Serial.println(",");
    Serial.print(Gx);Serial.print(",");
    Serial.print(Gy);Serial.print(",");
    Serial.print(Gz);Serial.println(",");
    Serial.print(depth);Serial.println();

    delay(10);
  }
}


// Function to calculate the delta time (dt_)
void CalculationDeltaTime() {
  // Get the current time
  time_now_ = micros();
  dt_ = (time_now_ - time_last_) / 1000000.0;  // Convert microseconds to seconds
  time_last_ = time_now_;  // Update the last time to the current time
}

// Function to calculate quaternion based on IMU data
void CalculationQuaternion(double gx, double gy, double gz, double ax, double ay, double az) {
    double recipNorm;
    double s0, s1, s2, s3;
    double qDot1, qDot2, qDot3, qDot4;
    double _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5 * (-q1_ * gx - q2_ * gy - q3_ * gz);
    qDot2 = 0.5 * (q0_ * gx + q2_ * gz - q3_ * gy);
    qDot3 = 0.5 * (q0_ * gy - q1_ * gz + q3_ * gx);
    qDot4 = 0.5 * (q0_ * gz + q1_ * gy - q2_ * gx);

    if (use_filter_) {
        // Compute feedback only if accelerometer measurement is valid (avoids NaN in accelerometer normalization)
        if (!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) {
            // Normalize accelerometer measurement
            recipNorm = sqrt(ax * ax + ay * ay + az * az);
            recipNorm = 1.0 / recipNorm;
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            _2q0 = 2.0 * q0_;
            _2q1 = 2.0 * q1_;
            _2q2 = 2.0 * q2_;
            _2q3 = 2.0 * q3_;
            _4q0 = 4.0 * q0_;
            _4q1 = 4.0 * q1_;
            _4q2 = 4.0 * q2_;
            _8q1 = 8.0 * q1_;
            _8q2 = 8.0 * q2_;
            q0q0 = q0_ * q0_;
            q1q1 = q1_ * q1_;
            q2q2 = q2_ * q2_;
            q3q3 = q3_ * q3_;

            // Gradient descent algorithm corrective step
            s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
            s1 = _4q1 * q3q3 - _2q3 * ax + 4.0 * q0q0 * q1_ - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
            s2 = 4.0 * q0q0 * q2_ + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
            s3 = 4.0 * q1q1 * q3_ - _2q1 * ax + 4.0 * q2q2 * q3_ - _2q2 * ay;
            recipNorm = sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // Normalize step magnitude
            recipNorm = 1.0 / recipNorm;
            s0 *= recipNorm;
            s1 *= recipNorm;
            s2 *= recipNorm;
            s3 *= recipNorm;

            // Apply feedback step
            qDot1 -= beta_ * s0;
            qDot2 -= beta_ * s1;
            qDot3 -= beta_ * s2;
            qDot4 -= beta_ * s3;
        }
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0_ += qDot1 * (1.0 * dt_);
    q1_ += qDot2 * (1.0 * dt_);
    q2_ += qDot3 * (1.0 * dt_);
    q3_ += qDot4 * (1.0 * dt_);

    // Normalize quaternion
    recipNorm = sqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
    recipNorm = 1.0 / recipNorm;
    q0_ *= recipNorm;
    q1_ *= recipNorm;
    q2_ *= recipNorm;
    q3_ *= recipNorm;
}
