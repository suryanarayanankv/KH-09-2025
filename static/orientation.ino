#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <math.h>  // for fabs

// Define I2C pins and other constants
#define SDA_PIN 21
#define SCL_PIN 22
#define CALIBRATION_SAMPLES 50

MPU6050 mpu;

// Global variables for accelerometer and orientation
float AccX, AccY, AccZ;
String currentOrientation = "Unknown";

// DMP-related variables
bool DMPReady = false;      
uint8_t devStatus;          
uint16_t packetSize;        
uint8_t FIFOBuffer[64];     
Quaternion q;  // Provided by MPU6050_6Axis_MotionApps20.h

//----------------------------------------------------------------------
// Function to read raw accelerometer data (for comparison or debugging)
//----------------------------------------------------------------------
void calculateWorldAccel() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);  // Starting register for accelerometer data
  Wire.endTransmission(); 
  Wire.requestFrom(0x68, 6);
  
  int16_t AccXLSB = (Wire.read() << 8) | Wire.read();
  int16_t AccYLSB = (Wire.read() << 8) | Wire.read();
  int16_t AccZLSB = (Wire.read() << 8) | Wire.read();
  
  // Convert accelerometer values to g's (assuming ±2g full scale)
  AccX = (float)AccXLSB / 16384.0;
  AccY = (float)AccYLSB / 16384.0;
  AccZ = (float)AccZLSB / 16384.0;
}

//----------------------------------------------------------------------
// (Existing) Function to determine orientation from accelerometer data
//----------------------------------------------------------------------
String determineOrientationFromAccel() {
  float absX = fabs(AccX);
  float absY = fabs(AccY);
  float absZ = fabs(AccZ);
  
  if (absX > absY && absX > absZ) {
    return (AccX > 0) ? "+X axis " : "-X axis ";
  } else if (absY > absX && absY > absZ) {
    return (AccY > 0) ? "+Y axis " : "-Y axis ";
  } else {
    return (AccZ > 0) ? "+Z axis " : "-Z axis ";
  }
}

//----------------------------------------------------------------------
// New function: Determine orientation using the quaternion
//----------------------------------------------------------------------
// This function rotates the hand's local forward vector (0,0,1) using the 
// quaternion (q) and then finds the dominant axis of the resulting vector.
String determineOrientationFromQuaternion() {
  // Rotate the local forward vector (0, 0, 1)
  // The rotated (global) vector is given by the third column of the rotation matrix:
  float forwardX = 2 * (q.x * q.z + q.y * q.w);
  float forwardY = 2 * (q.y * q.z - q.x * q.w);
  float forwardZ = 1 - 2 * (q.x * q.x + q.y * q.y);
  
  // Compute absolute values for comparison
  float absX = fabs(forwardX);
  float absY = fabs(forwardY);
  float absZ = fabs(forwardZ);
  
  // Determine the dominant axis and corresponding orientation
  if (absX > absY && absX > absZ) {
    return (forwardX > 0) ? "Away from you" : "Towrads you";
  } else if (absY > absX && absY > absZ) {
    return (forwardY > 0) ? "Left" : "Right";
  } else {
    return (forwardZ > 0) ? "Up" : "Down";
  }
}

//----------------------------------------------------------------------
// Setup function: Initialize I2C, MPU6050, and DMP
//----------------------------------------------------------------------
void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  
  Serial.begin(115200);
  while (!Serial); // Wait for Serial to be ready

  Serial.println(F("Initializing MPU6050..."));
  mpu.initialize();

  // Set accelerometer and gyro offsets and ranges as needed
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setXGyroOffset(2);
  mpu.setYGyroOffset(4);
  mpu.setZGyroOffset(1);
  mpu.setZAccelOffset(16384);

  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
    while (1);
  }

  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    DMPReady = true;
    packetSize = mpu.dmpGetFIFOPacketSize();
    Serial.println("\n=== MPU6050 Ready ===");
  } else {
    Serial.print("DMP Initialization failed (code ");
    Serial.print(devStatus);
    Serial.println(")");
  }
}

//----------------------------------------------------------------------
// Loop function: Read sensor data and determine hand orientation
//----------------------------------------------------------------------
void loop() {
  if (!DMPReady) return;
  
  // If a new packet is available from the DMP, process it.
  if (mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
    // Get the quaternion representing current orientation
    mpu.dmpGetQuaternion(&q, FIFOBuffer);
    
    // Also get accelerometer data (for comparison or debugging)
    calculateWorldAccel();
    
    // Print accelerometer values
    Serial.print("Acceleration (g):\tX=");
    Serial.print(AccX, 3);
    Serial.print("\tY=");
    Serial.print(AccY, 3);
    Serial.print("\tZ=");
    Serial.println(AccZ, 3);
    
    // Determine orientation from accelerometer (existing method)
    String accelOrientation = determineOrientationFromAccel();
    Serial.println("Orientation from Accel: " + accelOrientation);
    
    // Determine orientation from quaternion (new method)
    String quatOrientation = determineOrientationFromQuaternion();
    Serial.println("Orientation from Quaternion: " + quatOrientation);
    
    // Print the quaternion values for reference
    Serial.print("Quaternion (w, x, y, z):\t");
    Serial.print(q.w, 3); Serial.print("\t");
    Serial.print(q.x, 3); Serial.print("\t");
    Serial.print(q.y, 3); Serial.print("\t");
    Serial.println(q.z, 3);
  }
  
  delay(500);
}