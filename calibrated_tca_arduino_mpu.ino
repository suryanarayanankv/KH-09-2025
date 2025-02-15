#include "MPU6050_6Axis_MotionApps20.h"
#include <Wire.h>
#include <math.h>

#define TCAADDR 0x70
#define NUM_SENSORS 5
#define CHANNEL_SWITCH_DELAY 10

// Array of MPU objects for each sensor
MPU6050 mpus[NUM_SENSORS];

// Arrays to store accelerometer and orientation data for each sensor
float AccX[NUM_SENSORS], AccY[NUM_SENSORS], AccZ[NUM_SENSORS];
String currentOrientation[NUM_SENSORS];

// DMP-related variables for each sensor
bool DMPReady[NUM_SENSORS] = {false};
uint8_t devStatus[NUM_SENSORS];
uint16_t packetSize[NUM_SENSORS];
uint8_t FIFOBuffer[NUM_SENSORS][64];
Quaternion q[NUM_SENSORS];

// Error tracking
unsigned long lastSuccessfulRead = 0;

// Software reset function
void(* resetFunc) (void) = 0;

void tcaselect(uint8_t channel) {
    if (channel > 7) return;
    Wire.beginTransmission(TCAADDR);
    Wire.write(1 << channel);
    Wire.endTransmission();
    delay(CHANNEL_SWITCH_DELAY);  // Add consistent delay after switching
}

void checkAndResetFIFO(int sensorIndex) {
    tcaselect(sensorIndex);
    if (mpus[sensorIndex].getFIFOCount() >= 1024) {
        mpus[sensorIndex].resetFIFO();
        Serial.print(F("FIFO reset on sensor "));
        Serial.println(sensorIndex);
    }
}

void calculateWorldAccel(int sensorIndex) {
    tcaselect(sensorIndex);
    delay(10);
    
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 6);
    
    int16_t AccXLSB = (Wire.read() << 8) | Wire.read();
    int16_t AccYLSB = (Wire.read() << 8) | Wire.read();
    int16_t AccZLSB = (Wire.read() << 8) | Wire.read();
    
    AccX[sensorIndex] = (float)AccXLSB / 16384.0;
    AccY[sensorIndex] = (float)AccYLSB / 16384.0;
    AccZ[sensorIndex] = (float)AccZLSB / 16384.0;
}

String determineOrientationFromAccel(int sensorIndex) {
    float absX = fabs(AccX[sensorIndex]);
    float absY = fabs(AccY[sensorIndex]);
    float absZ = fabs(AccZ[sensorIndex]);
    
    if (absX > absY && absX > absZ) {
        return (AccX[sensorIndex] > 0) ? "+X axis " : "-X axis ";
    } else if (absY > absX && absY > absZ) {
        return (AccY[sensorIndex] > 0) ? "+Y axis " : "-Y axis ";
    } else {
        return (AccZ[sensorIndex] > 0) ? "+Z axis " : "-Z axis ";
    }
}

String determineOrientationFromQuaternion(int sensorIndex) {
    float forwardX = 2 * (q[sensorIndex].x * q[sensorIndex].z + q[sensorIndex].y * q[sensorIndex].w);
    float forwardY = 2 * (q[sensorIndex].y * q[sensorIndex].z - q[sensorIndex].x * q[sensorIndex].w);
    float forwardZ = 1 - 2 * (q[sensorIndex].x * q[sensorIndex].x + q[sensorIndex].y * q[sensorIndex].y);
    
    float absX = fabs(forwardX);
    float absY = fabs(forwardY);
    float absZ = fabs(forwardZ);
    
    if (absX > absY && absX > absZ) {
        return (forwardX > 0) ? "Away from you" : "Towards you";
    } else if (absY > absX && absY > absZ) {
        return (forwardY > 0) ? "Left" : "Right";
    } else {
        return (forwardZ > 0) ? "Up" : "Down";
    }
}

void setup() {
    Wire.begin();
    Wire.setClock(100000);  // Lower I2C speed for better stability
    
    Serial.begin(115200);
    while (!Serial);
    
    Serial.println(F("Initializing MPU6050 sensors..."));
    
    // Initialize each MPU6050
    for (int i = 0; i < NUM_SENSORS; i++) {
        tcaselect(i);
        delay(50);  // Give more time for switching and initialization
        
        Serial.print(F("Initializing MPU6050 on channel "));
        Serial.println(i);
        
        mpus[i].initialize();
        
        mpus[i].setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
        mpus[i].setXGyroOffset(2);
        mpus[i].setYGyroOffset(4);
        mpus[i].setZGyroOffset(1);
        mpus[i].setZAccelOffset(16384);
        
        if (mpus[i].testConnection()) {
            Serial.print(F("MPU6050 on channel "));
            Serial.print(i);
            Serial.println(F(" connection successful"));
            
            devStatus[i] = mpus[i].dmpInitialize();
            if (devStatus[i] == 0) {
                mpus[i].setDMPEnabled(true);
                DMPReady[i] = true;
                packetSize[i] = mpus[i].dmpGetFIFOPacketSize();
                Serial.print(F("DMP ready on channel "));
                Serial.println(i);
            } else {
                Serial.print(F("DMP Initialization failed on channel "));
                Serial.print(i);
                Serial.print(F(" (code "));
                Serial.print(devStatus[i]);
                Serial.println(F(")"));
            }
        } else {
            Serial.print(F("MPU6050 connection failed on channel "));
            Serial.println(i);
        }
    }
    
    lastSuccessfulRead = millis();
}

void loop() {
    // Process each sensor
    for (int i = 0; i < NUM_SENSORS; i++) {
        if (!DMPReady[i]) continue;
        
        tcaselect(i);
        delay(10);
        
        // Add timeout check
        unsigned long startTime = millis();
        bool dataRead = false;
        
        while ((millis() - startTime) < 100) {  // 100ms timeout
            if (mpus[i].dmpGetCurrentFIFOPacket(FIFOBuffer[i])) {
                dataRead = true;
                lastSuccessfulRead = millis();
                break;
            }
        }
        
        if (!dataRead) {
            checkAndResetFIFO(i);
            continue;
        }
        
        mpus[i].dmpGetQuaternion(&q[i], FIFOBuffer[i]);
        calculateWorldAccel(i);
        
        Serial.print(F("\n=== Sensor "));
        Serial.print(i);
        Serial.println(F(" ==="));
        
        Serial.print(F("Acceleration (g):\tX="));
        Serial.print(AccX[i], 3);
        Serial.print(F("\tY="));
        Serial.print(AccY[i], 3);
        Serial.print(F("\tZ="));
        Serial.println(AccZ[i], 3);
        
        Serial.print(F("Orientation from Accel: "));
        Serial.println(determineOrientationFromAccel(i));
        
        Serial.print(F("Orientation from Quaternion: "));
        Serial.println(determineOrientationFromQuaternion(i));
        
        Serial.print(F("Quaternion (w, x, y, z):\t"));
        Serial.print(q[i].w, 3); Serial.print(F("\t"));
        Serial.print(q[i].x, 3); Serial.print(F("\t"));
        Serial.print(q[i].y, 3); Serial.print(F("\t"));
        Serial.println(q[i].z, 3);
    }
    
    // Auto-reset if no successful reads for 5 seconds
    if (millis() - lastSuccessfulRead > 5000) {
        Serial.println(F("No data received for 5 seconds, resetting..."));
        resetFunc();
    }
    
    delay(100);  // Small delay between iterations
}