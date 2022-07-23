#ifndef IMU_H

#define IMU_H

/* Arduino library includes */ 
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/* Project specific includes */
#include "roverData.h" 
#include "roverPublish.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif
//------------------------------------------------------------------------------
/* 
 *  @brief: MPU-6050 Inertial Measurement Unit interface functions 
 *  
 *  @description: TODO
 *  
 *  @dependencies: This function requires https://github.com/jrowberg/i2cdevlib
 *  
 */
//------------------------------------------------------------------------------

/*
 * Init MPU6050 instance 
 */
MPU6050 mpu;

// Set Interrupt pin 
#define INTERRUPT_PIN 0

// Maximum value signed int16_t can carry 
#define MAX_INT_16 32767

/* 
 *  MPU control/status vars
 */
// set true if DMP init was successful
bool dmpReady = false;  

/*
 * Init IMU data instance 
 */
ImuData imuData;

/* 
 *  Holds actual interrupt status byte from MPU
 */
uint8_t mpuIntStatus;   
// return status after each device operation (0 = success, !0 = error)
uint8_t devStatus;      
// expected DMP packet size (default is 42 bytes)
uint16_t packetSize;    
// count of all bytes currently in FIFO
uint16_t fifoCount;     
// FIFO storage buffer
uint8_t fifoBuffer[64]; 

float acc_factor  = 4;
float gyro_factor = 250;

/* 
 *  Orientation/motion vars
 *  Temporary containers to get measurements
 */
// [w, x, y, z]         quaternion container
Quaternion q;           
// [x, y, z]            accel sensor measurements
VectorInt16 aa;         
// [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaReal;     
// [x, y, z]            world-frame accel sensor measurements
VectorInt16 aaWorld;    
// [x, y, z]            gravity vector
VectorFloat gravity;         
// [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float ypr[3];           

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

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

/*
 * IMU take measurement status flag constants 
 */
int IMU_MEAS_SUCCESS = 0;
int IMU_MEAS_FAILURE = 0;

/*
 * Set number of Acc/Gyro calibration loops
 * 1 loop ~= 100 measurements
 */
int ACC_CALIBRATION_LOOPS = 10;
int GYR_CALIBRATION_LOOPS = 10;

// ================================================================
// ===                    [ INIT ]                              ===
// ================================================================
int initIMU(){
  
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    // initialize device
    mpu.initialize();

    // load and configure the DMP
    publishData( String("[Wall-e][TM] Initializing DMP... "), MQTT_TOPIC_STATUS);

    /*
     * Test connection
     */
    if ( mpu.testConnection() ) {
      publishData( String("[Wall-e][TM] MPU6050 connection established. "), MQTT_TOPIC_STATUS);
    } else {
      publishData( String("[Wall-e][TM] MPU6050 connection failed. "), MQTT_TOPIC_STATUS);
    }

    delay(250);
    
    devStatus = mpu.dmpInitialize();

    /* 
     *  supply your own gyro offsets here, scaled for min sensitivity
     */
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // Calibration: generate offsets and calibrate our MPU6050
        /* Calibrate Accelerometer */
        mpu.CalibrateAccel(ACC_CALIBRATION_LOOPS);
        /* Calibrate Gyroscopes */
        mpu.CalibrateGyro(GYR_CALIBRATION_LOOPS);
        
        mpu.PrintActiveOffsets();
        
        /* Turn on the DMP, now that it's ready */
        publishData( String("[Wall-e][TM] Enabling DMP... "), MQTT_TOPIC_STATUS);
        
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        publishData( String("[Wall-e][TM] Enabling interrupt detection (Arduino external interrupt "), MQTT_TOPIC_STATUS);
        //attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //publishData( String("[Wall-e][TM] DMP ready! Waiting for first interrupt... "), MQTT_TOPIC_STATUS);
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        publishData( String("[Wall-e][TM] DMP Initialization failed (code "+String(devStatus)), MQTT_TOPIC_STATUS);

        return IMU_INIT_FAILURE;
    }
    
  imuData.vel_m_GRF.x = 0 ;
  imuData.vel_m_GRF.y = 0 ;
  imuData.vel_m_GRF.z = 0 ;
  
  imuData.pos_m_GRF.x = 0 ;
  imuData.pos_m_GRF.y = 0 ;
  imuData.pos_m_GRF.z = 0 ;

  /*
   * Set Sensitivity
   */
   /*
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  
  */
  gyro_factor = 250 * DEG2RAD;
  /*
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
  */
  acc_factor  = 4;
  
  return IMU_INIT_SUCCESS;
}

// ================================================================
// ===                    [ MEASUREMENT ]                       ===
// ================================================================
int updateImuMeasurement(){
  /* Init index for loops */
  int ii = 0 ;
  /*
   * If DMP programming failed -> exit on failure 
   */
  if (!dmpReady) {
    
    return IMU_MEAS_FAILURE;
  }
  
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { 

  // TODO: add temperature measurement
  imuData.temp_degC = -1;

  /*
   * @brief: Update quaternion
   */
  long meas_time = millis();

  imuData.delta_time_ms = meas_time - imuData.lastTimeTag_ms ;
  if ( imuData.delta_time_ms > imuData.DELTA_TIME_THR ) 
  {
    imuData.time_valid = imuData.TIME_INVALID;
  } else
  {
    imuData.time_valid = imuData.TIME_VALID;
  }
  imuData.lastTimeTag_ms = meas_time;
  
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetAccel(&aa, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
  mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  /*
   * @brief: Update Quaternion
   * @frame: Global reference frame to IMU frame
   * 
   */
  imuData.quat_Grf2Imu.w = q.w;
  imuData.quat_Grf2Imu.x = q.x;
  imuData.quat_Grf2Imu.y = q.y;
  imuData.quat_Grf2Imu.z = q.z;

  /*
   * @brief: Update Direct cosine matrix
   * @frame: Global reference frame to IMU frame
   * 
   */
  imuData.DCM_Grf2Imu[0][0] = ( q.w*q.w + q.x*q.x - q.y*q.y - q.z*q.z );
  imuData.DCM_Grf2Imu[1][0] = 2*( q.x*q.y - q.w*q.z ) ;
  imuData.DCM_Grf2Imu[2][0] = 2*( q.x*q.z + q.w*q.y ) ;

  imuData.DCM_Grf2Imu[0][1] = 2*( q.x*q.y + q.w*q.z );
  imuData.DCM_Grf2Imu[1][1] = ( q.w*q.w - q.x*q.x + q.y*q.y - q.z*q.z );
  imuData.DCM_Grf2Imu[2][1] = 2*( q.y*q.z - q.w*q.x );

  imuData.DCM_Grf2Imu[0][2] = 2*( q.x*q.z - q.w*q.y );
  imuData.DCM_Grf2Imu[1][2] = 2*( q.y*q.z + q.w*q.x );
  imuData.DCM_Grf2Imu[2][2] = ( q.w*q.w - q.x*q.x - q.y*q.y + q.z*q.z );

  /*
   * Create DCM for inverse rotation 
   */
  for ( ii=0 ; ii<3 ; ii++ )
  {
    imuData.DCM_Imu2Grf[0][ii] = imuData.DCM_Grf2Imu[ii][0];
    imuData.DCM_Imu2Grf[1][ii] = imuData.DCM_Grf2Imu[ii][1];
    imuData.DCM_Imu2Grf[2][ii] = imuData.DCM_Grf2Imu[ii][2];
  }

  /*
   * @brief: Update gravity vector 
   * @frame: IMU frame
   * @unit: m/s2
   */
  imuData.vec_grav_ms2_IMU.x = gravity.x;
  imuData.vec_grav_ms2_IMU.y = gravity.y;
  imuData.vec_grav_ms2_IMU.z = gravity.z;

  /*
   * @brief: Update Euler angles 
   * @unit: degree
   */
   /*
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  */
  imuData.euler_ypr_deg.yaw   = float( ypr[0] * ( RAD2DEG ));
  /*
  imuData.euler_ypr_deg.pitch = float( ypr[1] * ( RAD2DEG ));
  imuData.euler_ypr_deg.roll  = float( ypr[2] * ( RAD2DEG ));
  */

  
  /*
   * @brief: Update acceleration vector with removed gravity vector 
   * @frame: IMU frame
   * @unit: m/s2
   */
  imuData.acc_ms2_IMU.x = float(aaReal.x) / float( MAX_INT_16 * acc_factor );
  imuData.acc_ms2_IMU.y = float(aaReal.y) / float( MAX_INT_16 * acc_factor );
  imuData.acc_ms2_IMU.z = float(aaReal.z) / float( MAX_INT_16 * acc_factor );

  imuData.euler_ypr_deg.pitch  = float( asin( aa.x / sqrt( float( aa.x*aa.x + aa.y*aa.y + aa.z*aa.z))) * ( RAD2DEG ));
  imuData.euler_ypr_deg.roll   = float( atan2(aa.y,aa.z) * ( RAD2DEG ));

  if ( imuData.time_valid == imuData.TIME_VALID )
  {
    Vec3_float acc_ms2_GRF;

    /*
     * Create temporary vector: Acceleration in GRF frame (with removed gravity vector) 
     */
    imuData.acc_ms2_GRF.x = float(aaWorld.x) / float( MAX_INT_16 * acc_factor );
    imuData.acc_ms2_GRF.y = float(aaWorld.y) / float( MAX_INT_16 * acc_factor );
    imuData.acc_ms2_GRF.z = float(aaWorld.z) / float( MAX_INT_16 * acc_factor );

    /*
     * Update velocity in GRF frame
     */
    imuData.vel_m_GRF.x = imuData.vel_m_GRF.x + imuData.acc_ms2_IMU.x * float(imuData.delta_time_ms)/1000.0;
    imuData.vel_m_GRF.y = imuData.vel_m_GRF.y + imuData.acc_ms2_IMU.y * float(imuData.delta_time_ms)/1000.0;
    imuData.vel_m_GRF.z = imuData.vel_m_GRF.z + imuData.acc_ms2_IMU.z * float(imuData.delta_time_ms)/1000.0;
    
    /*
     * @brief: Update position 
     * @frame: Global reference frame
     * @unit: meter
     */
    imuData.pos_m_GRF.x = imuData.pos_m_GRF.x + (imuData.vel_m_GRF.x * float(imuData.delta_time_ms)/1000.0) + (0.5 * imuData.acc_ms2_IMU.x * (float(imuData.delta_time_ms)/1000.0) * (float(imuData.delta_time_ms)/1000.0));
    imuData.pos_m_GRF.y = imuData.pos_m_GRF.y + (imuData.vel_m_GRF.y * float(imuData.delta_time_ms)/1000.0) + (0.5 * imuData.acc_ms2_IMU.y * (float(imuData.delta_time_ms)/1000.0) * (float(imuData.delta_time_ms)/1000.0));
    imuData.pos_m_GRF.z = imuData.pos_m_GRF.z + (imuData.vel_m_GRF.z * float(imuData.delta_time_ms)/1000.0) + (0.5 * imuData.acc_ms2_IMU.z * (float(imuData.delta_time_ms)/1000.0) * (float(imuData.delta_time_ms)/1000.0));
  }  

  /*
   * Udpate Telemetry packet 
   */
  telemetry.imuData = imuData;

  return IMU_MEAS_SUCCESS;
  }
}

#endif
