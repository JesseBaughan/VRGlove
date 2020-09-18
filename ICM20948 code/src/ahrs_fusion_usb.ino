#include <Wire.h>
#include <Adafruit_Sensor.h>
//#include <Mahony_DPEng.h>
#include <Madgwick_DPEng.h>
#include <DPEng_ICM20948_AK09916.h>
#include <stdio.h>
#include <Arduino.h>


// Create sensor instance.
DPEng_ICM20948 dpEng = DPEng_ICM20948(0x948A, 0x948B, 0x948C);

// Mag calibration values are calculated via ahrs_calibration example sketch results.
// These values must be determined for each baord/environment.
// See the image in this sketch folder for the values used
// below.

// Offsets applied to raw x/y/z mag values
float mag_offsets[3]            = { -1.76F, 22.54F, 4.43F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.954,  -0.019,  0.003 },
                                    {  -0.019,  1.059, -0.009 },
                                    {  0.003,  -0.009,  0.990 } };

float mag_field_strength        = 29.85F;

// Offsets applied to compensate for gyro zero-drift error for x/y/z
float gyro_zero_offsets[3] = { 0.0F, 0.0F, 0.0F };
float accelOffsets[3] = { 0.0F, 0.0F, 0.0F };

// Mahony is lighter weight as a filter and should be used
// on slower systems
//Mahony_DPEng filter;
Madgwick_DPEng filter;

bool outputSerialToCSV = true;
bool outputAHRS = false;

void CalibrateIMU();

void setup()
{
  SerialUSB.begin(115200);

  // Wait for the SerialUSB Monitor to open (comment out to run without SerialUSB Monitor)
  while(!SerialUSB);

  SerialUSB.println(F("DPEng AHRS Fusion Example")); SerialUSB.println("");

  // Initialize the sensors.
  if(!dpEng.begin(ICM20948_ACCELRANGE_2G, GYRO_RANGE_250DPS))
  {
    /* There was a problem detecting the ICM20948 ... check your connections */
    SerialUSB.println("Ooops, no ICM20948/AK09916 detected ... Check your wiring!");
    while(1);
  }
  
  filter.begin();
  CalibrateIMU();
}

void loop(void)
{
  sensors_event_t accel_event;
  sensors_event_t gyro_event;
  sensors_event_t mag_event;

  // Get new data samples
  dpEng.getEvent(&accel_event, &gyro_event, &mag_event);

  // Apply mag offset compensation (base values in uTesla)
  float x = mag_event.magnetic.x - mag_offsets[0];
  float y = mag_event.magnetic.y - mag_offsets[1];
  float z = mag_event.magnetic.z - mag_offsets[2];

  // Apply mag soft iron error compensation
  float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
  float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
  float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

  // Apply gyro zero-rate error compensation
  float gx = gyro_event.gyro.x - gyro_zero_offsets[0];
  float gy = gyro_event.gyro.y - gyro_zero_offsets[1];
  float gz = gyro_event.gyro.z - gyro_zero_offsets[2];


  // Apply orientation offset for diff stating position
  float ax = accel_event.acceleration.x - accelOffsets[0];
  float ay = accel_event.acceleration.y - accelOffsets[1];
  float az = accel_event.acceleration.z + accelOffsets[2];

  // Update the filter
  filter.update(gx, gy, gz,
                ax, ay, az,
                mx, my*-1, mz*-1);
                
  float *qw;
  float *qx;
  float *qy; 
  float *qz;

  filter.getQuaternion(qw,qx,qy,qz);

  // Print the orientation filter output
  // Note: To avoid gimbal lock you should read quaternions not Euler
  // angles, but Euler angles are used here since they are easier to
  // understand looking at the raw values. See the ble fusion sketch for
  // and example of working with quaternion data.
  float roll = filter.getRoll();
  float pitch = filter.getPitch();
  float heading = filter.getYaw();
  
  static int counter = 0;

  //Wait for stabilisation
  if(counter > 2000) {
    //SerialUSB.print(millis());
    if(outputAHRS) {
      SerialUSB.print("Orientation: ");
      SerialUSB.print(heading);
      SerialUSB.print(" ");
      SerialUSB.print(pitch);
      SerialUSB.print(" ");
      SerialUSB.println(roll);
    }

    float gravity = 9.80665;

    if(outputSerialToCSV) {
      SerialUSB.print(ax/gravity, 6);
      SerialUSB.print(",");
      SerialUSB.print(ay/gravity, 6);
      SerialUSB.print(",");
      SerialUSB.print(az/gravity, 6);
      SerialUSB.print(",");
      SerialUSB.print(gx, 6);
      SerialUSB.print(",");
      SerialUSB.print(gy, 6);
      SerialUSB.print(",");
      SerialUSB.print(gz, 6);
      SerialUSB.print(",");
      SerialUSB.print(*qw, 6);
      SerialUSB.print(",");
      SerialUSB.print(*qx, 6);
      SerialUSB.print(",");
      SerialUSB.print(*qy, 6);
      SerialUSB.print(",");
      SerialUSB.print(*qz, 6);
      SerialUSB.println("");

    }
  }
   else {
    counter++;
  } 

  delay(10);
}

//Get offsets to compensate for diff starting orientations
void CalibrateIMU() {
  uint16_t numSamples = 1000;
  uint16_t startSampling = 500; //Wait seconds for rigger to move hand off device
  float axSum = 0;
  float aySum = 0;
  float azSum = 0;
  float gxSum = 0;
  float gySum = 0;
  float gzSum = 0;

  sensors_event_t accel_event;
  sensors_event_t gyro_event;
  sensors_event_t mag_event;

  SerialUSB.println("Getting orientation offsets...this make take up to 30s");

  int start = millis();

  for (int i = 0; i < numSamples; i++) {
    if (i > startSampling) {
      // Get new data samples
      dpEng.getEvent(&accel_event, &gyro_event, &mag_event);

      // Apply gyro zero-rate error compensation
      float gx = gyro_event.gyro.x;
      float gy = gyro_event.gyro.y;
      float gz = gyro_event.gyro.z;

      // Apply orientation offset for diff stating position
      float ax = accel_event.acceleration.x;
      float ay = accel_event.acceleration.y;
      float az = accel_event.acceleration.z;

      axSum += ax; 
      aySum += ay;
      azSum += az;
      gxSum += gx; 
      gySum += gy;
      gzSum += gz;
      
/*       SerialUSB.print(gx);
      SerialUSB.print(" ");
      SerialUSB.print(gy);
      SerialUSB.print(" ");
      SerialUSB.println(gz); */

      } 
    delay(10); 
  }

  accelOffsets[0] = axSum/(numSamples - startSampling);
  accelOffsets[1] = aySum/(numSamples - startSampling);
  accelOffsets[2] = azSum/(numSamples - startSampling);
  accelOffsets[2] = 9.80665 - abs(accelOffsets[2]);

  gyro_zero_offsets[0] = gxSum/(numSamples - startSampling);
  gyro_zero_offsets[1] = gySum/(numSamples - startSampling);
  gyro_zero_offsets[2] = gzSum/(numSamples - startSampling);

/*   SerialUSB.print(accelOffsets[0]);
  SerialUSB.print(" ");
  SerialUSB.print(accelOffsets[1]);
  SerialUSB.print(" ");
  SerialUSB.print(accelOffsets[2]);
  SerialUSB.print(" ");
  SerialUSB.print(gyro_zero_offsets[0]);
  SerialUSB.print(" ");
  SerialUSB.print(gyro_zero_offsets[1]);
  SerialUSB.print(" ");
  SerialUSB.println(gyro_zero_offsets[2]); */

  return;
}
