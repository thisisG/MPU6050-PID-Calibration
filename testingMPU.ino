#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "PID_v1.h"

// Uncomment to provide human readable output
//#define HUMAN_READABLE_OUTPUT
// Uncomment to provide parsable output in CSV format
#define PARSABLE_OUTPUT
// Uncomment to provide debug ouput
//#define DEBUG_OUTPUT

// Use arrays to simplify the code
const int arraySize = 6;
const int avgArraySize = 15;
// PID uses doubles, MPU commands are int16_ts
int16_t receivedData[arraySize] = { 0 };
double dreceivedData[arraySize] = { 0 };
int16_t offsets[arraySize] = { 0 };
double doffsets[arraySize] = { 0 };
// Store values in a circular buffer to calculate averages
int avgIndexCount[arraySize] = { 0 };
double offsetArray[arraySize][avgArraySize] = { 0 };
double readValueArray[arraySize][avgArraySize] = { 0 };

// Set point for XY axis is 0 if perfectly level
double XYAccSetpoint = 0;
// Set point for Z axis is 16384(gravity)
double ZAccSetpoint = 16384;
// Set point for gyros are 0
double gyroSetpoint = 0;
// Use array of setpoints to ease the setup process
double dsetpoint[arraySize] = { XYAccSetpoint, XYAccSetpoint, ZAccSetpoint,
                                gyroSetpoint,  gyroSetpoint,  gyroSetpoint };

// Set tunings
double kp = 0.03125;
double ki = 0.25;
double kd = 0;
// Array of pointers to PIDs used throughout
PID tuningController[arraySize];

// MPU to calibrate, using int0 for interrupts
MPU6050 mpu;

// Forward declarations
void switchAndUpdateOffsets(const size_t index);
double avgOffsetFromIndex(const size_t index);
double avgValueFromIndex(const size_t index);
void printCalibrationDataToSerial();

void setup()
{
  // Enable serial
  Serial.begin(115200);

  // Setup the PID controllers
  for (size_t i = 0; i < arraySize; ++i)
  {
    tuningController[i].setup(&(dreceivedData[i]), &(doffsets[i]),
                              &(dsetpoint[i]), kp, ki, kd, DIRECT);
    tuningController[i].SetOutputLimits(-15000, 15000);
    tuningController[i].SetMode(AUTOMATIC);
  }
  // Start the I2C bus
  Wire.begin();

  // Start the MPU
#ifdef DEBUG_OUTPUT
  Serial.println(F("Initializing MPU"));
#endif // DEBUG_OUTPUT
  mpu.initialize();
  // Wait for a bit while initialising
  delay(10);
  // Check if connection is up
  if (!mpu.testConnection())
  {
#ifdef DEBUG_OUTPUT
    Serial.println(F("MPU connection failed, reset it!"));
#endif // DEBUG_OUTPUT
  }
}

void loop()
{
  static unsigned long lastSerialUpdate = 0;
  static const unsigned long deltaTSerialUpdate = 200;
  // Get data from MPU
  // [0]: xAccelerometer
  // [1]: yAccelerometer
  // [2]: zAccelerometer
  // [3]: xGyro
  // [4]: yGyro
  // [5]: zGyro
  mpu.getMotion6(&receivedData[0], &receivedData[1], &receivedData[2],
                 &receivedData[3], &receivedData[4], &receivedData[5]);

  // Convert to doubles for PIDs
  for (size_t i = 0; i < arraySize; ++i)
  {
    dreceivedData[i] = static_cast<double>(receivedData[i]);
  }

  // Check and update PIDs
  for (size_t i = 0; i < arraySize; ++i)
  {
    if (tuningController[i].Compute())
    {
      switchAndUpdateOffsets(i);
      // Store the most recently read value and new offset to averaging arrays
      offsetArray[i][avgIndexCount[i]] = doffsets[i];
      readValueArray[i][avgIndexCount[i]] = dreceivedData[i];
      avgIndexCount[i]++;
      if (avgIndexCount[i] >= avgArraySize)
      {
        avgIndexCount[i] = 0;
      }
    }
  }

  // Check if time to send update of average values over serial
  if ((millis() - lastSerialUpdate) > deltaTSerialUpdate)
  {
    lastSerialUpdate = millis();
    printCalibrationDataToSerial();
  }
}

void switchAndUpdateOffsets(const size_t index)
{
  offsets[index] = static_cast<int16_t>(doffsets[index]);
  switch (index)
  {
  case 0:
    mpu.setXAccelOffset(offsets[index]);
    break;
  case 1:
    mpu.setYAccelOffset(offsets[index]);
    break;
  case 2:
    mpu.setZAccelOffset(offsets[index]);
    break;
  case 3:
    mpu.setXGyroOffset(offsets[index]);
    break;
  case 4:
    mpu.setYGyroOffset(offsets[index]);
    break;
  case 5:
    mpu.setZGyroOffset(offsets[index]);
    break;
  default:
    break;
  }
}

double avgOffsetFromIndex(const size_t index)
{
  double sum = 0;
  for (size_t i = 0; i < avgArraySize; ++i)
  {
    sum += offsetArray[index][i];
  }

  return (sum / static_cast<double>(avgArraySize));
}

double avgValueFromIndex(const size_t index)
{
  double sum = 0;
  for (size_t i = 0; i < avgArraySize; ++i)
  {
    sum += readValueArray[index][i];
  }

  return (sum / static_cast<double>(avgArraySize));
}

void printCalibrationDataToSerial()
{
#ifdef HUMAN_READABLE_OUTPUT
  Serial.println(
      F("avgXacc \tavgYacc \tavgZacc \tavgXgyro \tavgYgyro \tavgZgyro"));
  for (size_t i = 0; i < arraySize; ++i)
  {
    Serial.print(avgValueFromIndex(i), 1);
    Serial.print("\t\t");
  }
  Serial.println("");
  Serial.println(F("avgXaccOff \tavgYaccOff \tavgZaccOff \tavgXgyroOff \t"
                   "avgXgyroOff \tavgXgyroOff"));
  for (size_t i = 0; i < arraySize; ++i)
  {
    Serial.print(avgOffsetFromIndex(i), 1);
    Serial.print("\t\t");
  }
  Serial.println("");
#endif // HUMAN_READABLE_OUTPUT

#ifdef PARSABLE_OUTPUT
  for (size_t i = 0; i < arraySize-1; ++i)
  {
    Serial.print(avgValueFromIndex(i), 1);
    Serial.print(",");
    Serial.print(avgOffsetFromIndex(i),1);
    Serial.print(",");
  }
  Serial.print(avgValueFromIndex(arraySize - 1), 1);
  Serial.print(",");
  Serial.println(avgOffsetFromIndex(arraySize - 1), 1);
#endif // PARSABLE_OUTPUT
}
