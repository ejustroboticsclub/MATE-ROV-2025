#include <Wire.h>

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// MPU6050 registers
#define MPU6050_ACCEL_XOUT_H 0x3B
#define MPU6050_GYRO_XOUT_H  0x43
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_WHO_AM_I     0x75

// Variables for sensor readings
int16_t accelX, accelY, accelZ;
int16_t gyroX, gyroY, gyroZ;
int16_t temperature;

// For calculating actual values
const float accelScale = 16384.0;  // for ±2g range
const float gyroScale = 131.0;     // for ±250°/s range

unsigned long lastTime = 0;
unsigned long sampleRate = 100;  // 100ms = 10Hz

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect
  }
  
  // Initialize I2C
  Wire.begin();
  
  // Check if MPU6050 is connected
  byte whoAmI = readMPU6050Register(MPU6050_WHO_AM_I);
  if (whoAmI != 0x68) {
    Serial.print("Expected MPU6050 WHO_AM_I to be 0x68, got 0x");
    Serial.println(whoAmI, HEX);
    Serial.println("Check your connections!");
    while (1) {
      delay(1000);
    }
  }
  
  // Initialize MPU6050
  initMPU6050();
  
  Serial.println("MPU6050 initialized successfully!");
  Serial.println("----------------------------------");
  Serial.println("Raw Values | Converted Values (g, °/s)");
  Serial.println("----------------------------------");
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read at the specified sample rate
  if (currentTime - lastTime >= sampleRate) {
    lastTime = currentTime;
    
    // Read all sensor data
    readMPU6050Data();
    
    // Print raw data
    Serial.print("ACCEL X: ");
    Serial.print(accelX);
    Serial.print("\tY: ");
    Serial.print(accelY);
    Serial.print("\tZ: ");
    Serial.print(accelZ);
    
    // Print converted acceleration data (in g)
    Serial.print("\t| X: ");
    Serial.print(accelX / accelScale, 2);
    Serial.print("g\tY: ");
    Serial.print(accelY / accelScale, 2);
    Serial.print("g\tZ: ");
    Serial.print(accelZ / accelScale, 2);
    Serial.println("g");
    
    // Print raw gyro data
    Serial.print("GYRO  X: ");
    Serial.print(gyroX);
    Serial.print("\tY: ");
    Serial.print(gyroY);
    Serial.print("\tZ: ");
    Serial.print(gyroZ);
    
    // Print converted gyro data (in degrees/second)
    Serial.print("\t| X: ");
    Serial.print(gyroX / gyroScale, 2);
    Serial.print("°/s\tY: ");
    Serial.print(gyroY / gyroScale, 2);
    Serial.print("°/s\tZ: ");
    Serial.print(gyroZ / gyroScale, 2);
    Serial.println("°/s");
    
    // Print divider
    Serial.println("----------------------------------");
  }
}

void initMPU6050() {
  // Wake up MPU6050 (clear sleep bit)
  writeMPU6050Register(MPU6050_PWR_MGMT_1, 0x00);
  
  // Small delay for the sensor to initialize
  delay(100);
}

void readMPU6050Data() {
  // Request accelerometer data (6 bytes)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_ACCEL_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  
  // Read accelerometer data (high byte first, then low byte)
  accelX = (Wire.read() << 8) | Wire.read();
  accelY = (Wire.read() << 8) | Wire.read();
  accelZ = (Wire.read() << 8) | Wire.read();
  
  // Request gyroscope data (6 bytes)
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(MPU6050_GYRO_XOUT_H);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 6, true);
  
  // Read gyroscope data (high byte first, then low byte)
  gyroX = (Wire.read() << 8) | Wire.read();
  gyroY = (Wire.read() << 8) | Wire.read();
  gyroZ = (Wire.read() << 8) | Wire.read();
}

void writeMPU6050Register(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readMPU6050Register(uint8_t reg) {
  Wire.beginTransmission(MPU6050_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU6050_ADDR, 1, true);
  return Wire.read();
}