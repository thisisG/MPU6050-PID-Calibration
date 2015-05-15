#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include "PID_v1.h"

MPU6050 mpu;

// Use arrays to simplify the code
const int arraySize = 6;

int16_t receivedData[arraySize];
double dreceivedData[arraySize];

int16_t offsets[arraySize] = { 0 };
double doffsets[arraySize] = { 0 };

// Set point for XY axis is 0 if perfectly level
double XYAccSetPoint = 0;
// Set point for Z axis is 16384(gravity)
double ZAccSetPoint = 16384;

// Use array of setpoints to ease the setup process
double dsetpoint[arraySize]
    = { XYAccSetPoint, XYAccSetPoint, ZAccSetPoint, 0, 0, 0 };

// Set tunings
double kp = 0.03125;
double ki = 0.25;
double kd = 0;

// Set up PID linked to x axis accelerometer
PID* tuningController[arraySize] = { NULL };

void setup()
{
  // Create and setup the PID controllers
  for (size_t i = 0; i < arraySize; i++)
  {
    tuningController[i] = new PID(&(dreceivedData[i]), &(doffsets[i]),
                                  &(dsetpoint[i]), kp, ki, kd, DIRECT);
    tuningController[i]->SetOutputLimits(-15000, 15000);
    tuningController[i]->SetMode(AUTOMATIC);
  }
  Wire.begin();

  // Enable serial
  Serial.begin(115200);

  // Start the MPU
  Serial.println("Initializing MPU");
  mpu.initialize();

  // Wait for a bit while initialising
  delay(10);

  // Check if connection is up, if not retry
  while (!mpu.testConnection())
  {
    delay(100);
    Serial.println("Retrying MPU connection");
    mpu.initialize();
  }
}

void loop()
{
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
  for (size_t i = 0; i < arraySize; i++)
  {
    dreceivedData[i] = static_cast<double>(receivedData[i]);
  }

  // Check and update PIDs
  for (size_t i = 0; i < arraySize; i++)
  {
    if (tuningController[i]->Compute())
    {
      offsets[i] = static_cast<int16_t>(doffsets[i]);
      switch (i)
      {
      case 0:
        mpu.setXAccelOffset(offsets[i]);
        break;
      case 1:
        mpu.setYAccelOffset(offsets[i]);
        break;
      case 2:
        mpu.setZAccelOffset(offsets[i]);
        break;
      case 3:
        mpu.setXGyroOffset(offsets[i]);
        break;
      case 4:
        mpu.setYGyroOffset(offsets[i]);
        break;
      case 5:
        mpu.setZGyroOffset(offsets[i]);
        break;
      default:
        break;
      }
      // Output acceleration x axis and offset
      Serial.print("receivedData[i]: ");
      Serial.print(receivedData[i]);
      Serial.print(", offsets[i]: ");
      Serial.println(offsets[i]);
    }
  }
}
