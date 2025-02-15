

#include "I2Cdev.h"
#include "MPU6050.h"

/* Create the class for MPU6050. Default I2C address is 0x68 */
MPU6050 mpu;
// MPU6050 mpu(0x69); // <-- use for AD0 high

const int usDelay = 3150; // Delay in ms to hold the sampling at 200Hz
const int NFast = 1000; // Number of quick readings for averaging, the higher the better
const int NSlow = 10000; // Number of slow readings for averaging, the higher the better
const int LinesBetweenHeaders = 5;

const int iAx = 0;
const int iAy = 1;
const int iAz = 2;
const int iGx = 3;
const int iGy = 4;
const int iGz = 5;

int LowValue[6];
int HighValue[6];
int Smoothed[6];
int LowOffset[6];
int HighOffset[6];
int Target[6];
int LinesOut;
int N;
int i;

void setup() {
  Initialize(); //Initializate and calibrate the sensor
  for (i = iAx; i <= iGz; i++) { 
    Target[i] = 0; // Fix for ZAccel 
    HighOffset[i] = 0;
    LowOffset[i] = 0;
  } 
  Target[iAz] = 16384; // Set the taget for Z axes
  SetAveraging(NFast); // Fast averaging
  PullBracketsOut();
  PullBracketsIn();
  Serial.println("-------------- DONE --------------");
}

void loop() {
  //Write your code here
} 

/*Initializate function*/
void Initialize() {
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  Serial.begin(9600);
  // Init the module
  Serial.println("Initializing MPU...");
  mpu.initialize();
  Serial.println("MPU initializated");
  // Check module connection
  Serial.println("Testing device connections...");
  if(mpu.testConnection() ==  false){
    Serial.println("MPU6050 connection failed");
  while(true);
  }
  else{
    Serial.println("MPU6050 connection successful");
  }

  Serial.println("\nPID tuning Each Dot = 100 readings");

  Serial.println("\nXAccel\t\tYAccel\t\tZAccel\t\tXGyro\t\tYGyro\t\tZGyro");
  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  Serial.println("\n600 Readings");
  mpu.PrintActiveOffsets();
  mpu.CalibrateAccel(1);
  mpu.CalibrateGyro(1);
  Serial.println("700 Total Readings");
  mpu.PrintActiveOffsets();
  mpu.CalibrateAccel(1);
  mpu.CalibrateGyro(1);
  Serial.println("800 Total Readings");
  mpu.PrintActiveOffsets();
  mpu.CalibrateAccel(1);
  mpu.CalibrateGyro(1);
  Serial.println("900 Total Readings");
  mpu.PrintActiveOffsets();
  mpu.CalibrateAccel(1);
  mpu.CalibrateGyro(1);
  Serial.println("1000 Total Readings");
  mpu.PrintActiveOffsets();
  Serial.println("\nAny of the above offsets will work nicely \n\nProving the PID with other method:");
}

void SetAveraging(int NewN) {
  N = NewN;
  Serial.print("\nAveraging ");
  Serial.print(N);
  Serial.println(" readings each time");
}

void PullBracketsOut() {
  boolean Done = false;
  int NextLowOffset[6];
  int NextHighOffset[6];

  Serial.println("Expanding:");
  ForceHeader();

  while (!Done) {
    Done = true;
    SetOffsets(LowOffset); //Set low offsets
    GetSmoothed();
    for (i = 0; i <= 5; i++) { // Get low values
      LowValue[i] = Smoothed[i];
      if (LowValue[i] >= Target[i]) {
        Done = false;
        NextLowOffset[i] = LowOffset[i] - 1000;
      } 
      else {
        NextLowOffset[i] = LowOffset[i];
      }
    }
    SetOffsets(HighOffset);
    GetSmoothed();
    for (i = 0; i <= 5; i++) { // Get high values
      HighValue[i] = Smoothed[i];
      if (HighValue[i] <= Target[i]) {
        Done = false;
        NextHighOffset[i] = HighOffset[i] + 1000;
      } 
      else {
        NextHighOffset[i] = HighOffset[i];
      }
    } 
    ShowProgress();
    for (int i = 0; i <= 5; i++) {
      LowOffset[i] = NextLowOffset[i]; 
      HighOffset[i] = NextHighOffset[i];
    }
  }
}

void PullBracketsIn() {
  boolean AllBracketsNarrow;
  boolean StillWorking;
  int NewOffset[6];

  Serial.println("\nClosing in:");
  AllBracketsNarrow = false;
  ForceHeader();
  StillWorking = true;
  while (StillWorking) {
    StillWorking = false;
    if (AllBracketsNarrow && (N == NFast)) {
      SetAveraging(NSlow);
    } 
    else {
      AllBracketsNarrow = true;
    }
    for (int i = 0; i <= 5; i++) {
      if (HighOffset[i] <= (LowOffset[i] + 1)) {
        NewOffset[i] = LowOffset[i];
      } 
      else { // Binary search
        StillWorking = true;
        NewOffset[i] = (LowOffset[i] + HighOffset[i]) / 2;
        if (HighOffset[i] > (LowOffset[i] + 10)) {
          AllBracketsNarrow = false;
        }
      } 
    }
    SetOffsets(NewOffset);
    GetSmoothed();
    for (i = 0; i <= 5; i++) { // Closing in
      if (Smoothed[i] > Target[i]) { // Use lower half
        HighOffset[i] = NewOffset[i];
        HighValue[i] = Smoothed[i];
      } 
      else { // Use upper half
        LowOffset[i] = NewOffset[i];
        LowValue[i] = Smoothed[i];
      } 
    }
    ShowProgress();
  } 
} 

void ForceHeader() {
  LinesOut = 99;
}

/*Function to smooth the read values*/
void GetSmoothed() {
  int16_t RawValue[6];
  long Sums[6];
  for (i = 0; i <= 5; i++) {
    Sums[i] = 0;
  }
  
/* Get Sums*/
  for (i = 1; i <= N; i++) { 
    mpu.getMotion6( & RawValue[iAx], & RawValue[iAy], & RawValue[iAz], & RawValue[iGx], & RawValue[iGy], & RawValue[iGz]);
    delayMicroseconds(usDelay);
    for (int j = 0; j <= 5; j++){
      Sums[j] = Sums[j] + RawValue[j];
    }
  } 
  for (i = 0; i <= 5; i++) {
    Smoothed[i] = (Sums[i] + N / 2) / N;
  }
} 

/*Function for configure the oba=tained offsets*/
void SetOffsets(int TheOffsets[6]) {
  mpu.setXAccelOffset(TheOffsets[iAx]);
  mpu.setYAccelOffset(TheOffsets[iAy]);
  mpu.setZAccelOffset(TheOffsets[iAz]);
  mpu.setXGyroOffset(TheOffsets[iGx]);
  mpu.setYGyroOffset(TheOffsets[iGy]);
  mpu.setZGyroOffset(TheOffsets[iGz]);
}

/*Print the progress of the reading averages, add formatting for better visualization*/
void ShowProgress() {
 /*Header*/
  if (LinesOut >= LinesBetweenHeaders) { 
    Serial.println("\t\tXAccel\t\t\tYAccel\t\t\t\tZAccel\t\t\tXGyro\t\t\tYGyro\t\t\tZGyro");
    LinesOut = 0;
  } 
  Serial.print(' ');
  for (i = 0; i <= 5; i++) {
    Serial.print('[');
    Serial.print(LowOffset[i]),
    Serial.print(',');
    Serial.print(HighOffset[i]);
    Serial.print("] --> [");
    Serial.print(LowValue[i]);
    Serial.print(',');
    Serial.print(HighValue[i]);
    if (i == 5) {
      Serial.println("]");
    } 
    else {
      Serial.print("]\t");
    }
  }
  LinesOut++;
} 
