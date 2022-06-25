#ifndef IMU_H

#define IMU_H

/* Arduino library includes */ 
#include <Adafruit_MPU6050.h>

/* Project specific includes */
#include "roverData.h" 
//------------------------------------------------------------------------------
/* 
 *  @brief: MPU-6050 Inertial Measurement Unit interface functions 
 *  
 *  @description: TODO
 *  
 */
Adafruit_MPU6050 mpu;
Adafruit_Sensor *mpu_temp, *mpu_accel, *mpu_gyro;

#define PI 3.14159265359

double mean_gx = 0;
double mean_gy = 0;
double mean_gz = 0;

double off_gx = 0;
double off_gy = 0;
double off_gz = 0;

/*
 * Gyro calibration buffer size:
 * Number of static IMU measurements taken to determine 
 * the static measurement offsets
 */
int GYRO_CALIB_BUFFER_SIZE = 1000;

/*
 * IMU init status flag constants 
 */
int IMU_INIT_SUCCESS = 0;
int IMU_INIT_FAILURE = 1; 
/*
 * Maximum attempts to init MPU
 * If threshold is reached -> giving up 
 */
int MAX_INIT_ATTEMPTS = 10;

void calibrateGyro(){
  int i;
  int bufferSize = 1000;
  
  sensors_event_t gyro;
  for(i=0;i<GYRO_CALIB_BUFFER_SIZE;i++)
  {
      mpu_gyro->getEvent(&gyro);

      mean_gx = mean_gx + gyro.gyro.x;
      mean_gy = mean_gy + gyro.gyro.y;
      mean_gz = mean_gz + gyro.gyro.z;

      delay(2);
  }

  /*
   * Finalize mean gyro measurements per axis 
   */
  mean_gx = double( mean_gx / GYRO_CALIB_BUFFER_SIZE );
  mean_gy = double( mean_gy / GYRO_CALIB_BUFFER_SIZE );
  mean_gz = double( mean_gz / GYRO_CALIB_BUFFER_SIZE );

  /*
   * Generate axis offsets 
   */
  off_gx = - mean_gx ;
  off_gy = - mean_gy ;
  off_gz = - mean_gz ;
}

double rad2deg(double angle_rad){
  return (angle_rad * 180 / PI );
}

double deg2rad(double angle_deg){
  return (angle_deg * PI / 180);
}

int initIMU(){
  int loop_counter = 0 ; 
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
      if ( loop_counter == MAX_INIT_ATTEMPTS ) 
      {
        return IMU_INIT_FAILURE;
      }
      loop_counter = loop_counter + 1;
    }
  }

  mpu_temp = mpu.getTemperatureSensor();
  //mpu_temp->printSensorDetails();

  mpu_accel = mpu.getAccelerometerSensor();
  //mpu_accel->printSensorDetails();

  mpu_gyro = mpu.getGyroSensor();
  //mpu_gyro->printSensorDetails();

  /*
   * MPU-6050 Settings
   */
  //mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  //mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  //mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);

  /*
   * Determine static gyro measurement offsets for each axis
   */
  calibrateGyro();

  return IMU_INIT_SUCCESS;
}

ImuData updateImuMeasurement(){
  ImuData imuData;
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  mpu_temp->getEvent(&temp);
  mpu_accel->getEvent(&accel);
  mpu_gyro->getEvent(&gyro);

  /*
   * Update measurement time related parameters
   */
  long prev_timeTag = imuData.timeTag;
  int ii = 0;
  imuData.timeTag = millis();
  imuData.delta_time_ms = imuData.timeTag - prev_timeTag ;

  /*
   * Update linear acceleration measurements 
   * @Unit [m/s2]
   */
  imuData.ax = accel.acceleration.x;
  imuData.ay = accel.acceleration.y;
  imuData.az = accel.acceleration.z;

  /*
   * Update angular velocity 
   * * @Unit [rad/s]
   */
  imuData.gx = gyro.gyro.x + off_gx;
  imuData.gy = gyro.gyro.y + off_gy;
  imuData.gz = gyro.gyro.z + off_gz;

  /*
   * Update angle delta since last measurement
   */
  imuData.delta_axis_angle_rad[0] = gyro.gyro.x * double( double(imuData.delta_time_ms) / 1000 );
  imuData.delta_axis_angle_rad[1] = gyro.gyro.y * double( double(imuData.delta_time_ms) / 1000 );
  imuData.delta_axis_angle_rad[2] = gyro.gyro.z * double( double(imuData.delta_time_ms) / 1000 );

  /*
   * Update Euler Angles Yaw Pitch Roll
   */
  for (ii=0;ii<3;ii++)
  {
    imuData.euler_ypr[ii] = double( double(imuData.euler_ypr[ii]) + rad2deg( double(imuData.delta_axis_angle_rad[ii]) ) );
  }

  /*
   * Update temperature measure 
   * @Unit [deg Celsius]
   */
  imuData.temperature = temp.temperature;

  telemetry.imuData = imuData;
  return imuData;
}

#endif
