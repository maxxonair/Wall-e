#ifndef ROVER_DATA_H

#define ROVER_DATA_H

#include "roverSettings.h"
#include "roverPid.h"

#define PI 3.14159265359

#define RAD2DEG (180/PI)

#define DEG2RAD (PI/180)


 /*
  * Core ID at which Telemetry task will be executed
  */
 static int TM_TASK_AT_CORE_ID   =   1;
// ================================================================
// ===                   [ WIFI ]                               ===
// ================================================================
// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;

// Set static IP address
IPAddress local_IP(192, 168, 1, 189); 
// Set Gateway IP address
IPAddress gateway(192, 168, 1, 1);

IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(79, 79, 79, 77);   //optional
IPAddress secondaryDNS(79, 79, 79, 78); //optional

const String thisHostname = "walle";

// ================================================================
// ===                   [ INTERNAL STATUS ]                    ===
// ================================================================
/*
 * @brief: Counter for wifi connection attempt failures
 */
long connection_failures        = 0;
/*
 * @brief: Counter for mqtt broker connection attempt failures
 */
long broker_connection_failures = 0;
// ================================================================
// ===                     [ MQTT ]                             ===
// ================================================================
const char* mqtt_server       = "192.168.1.198";  
const char* mqtt_username     = "box"; 
const char* mqtt_password     = "box"; 
const char* client_id         = "client_roomBox_01"; 

const int port = 1883;

// Topics Publish
const char* MQTT_TOPIC_DISTANCE     = "wallE/distance";
const char* MQTT_TOPIC_OPEN_MIC     = "wallE/openMic";
const char* MQTT_TOPIC_STATUS       = "wallE/status";
const char* MQTT_TOPIC_TM           = "wallE/TM";
const char* MQTT_TOPIC_TC           = "wallE/TC";

// Mqtt status 
static bool hasIoTHub     = false;
// Enable MQTT debug prints
static bool isMqttDebug   = true;

//PubSubClient client(wifiClient);
PubSubClient client(mqtt_server, port, wifiClient); 
// ================================================================
// ===                   [ CONTROLLER ]                         ===
// ================================================================
PidData heading_ctrl;

void setupHeadingController(){
  heading_ctrl.P_GAIN = 1.1;
  heading_ctrl.I_GAIN = 0.001;
  heading_ctrl.D_GAIN = 0.9;
  
  heading_ctrl.CMD_MAX                 = 255;
  heading_ctrl.CMD_MIN                 =   0;
  heading_ctrl.enable_boundary_control =   1;
  
  ctrl_reset(heading_ctrl);
}

//------------------------------------------------------------------------------
// >> Constants
//------------------------------------------------------------------------------
String TC_IDENTIFIER = "[TC]";
String TC_SET_VEHICLE_MODE              = "SET_VEHICLE_MODE";
String TC_BROADCAST_DIST                = "SET_BROADCAST_DIST";
String TC_SET_AUTO_PACE                 = "SET_AUTO_PACE";
String TC_ENABLE_TM                     = "SET_ENABLE_TM";
/* Point turn in counter clockwise direction */
String TC_POINT_TURN_1                  = "SET_PT_1";
/* Point turn in clockwise direction */
String TC_POINT_TURN_0                  = "SET_PT_0";
String TC_GO_FORWARD                    = "SET_GO_FORWARD";
String TC_GO_REVERSE                    = "SET_GO_REVERSE";
String TC_GO_POINT_TURN_CT              = "SET_GO_PT_CT";
String TC_SET_PID_P                     = "SET_PID_P";
String TC_SET_PID_I                     = "SET_PID_I";
String TC_SET_PID_D                     = "SET_PID_D";
/*
 * Task handle for telemetry task running on the second core 
 */
TaskHandle_t TelemetryTask;
/*
 * Maximum allowable point turn duration when commanded via TC [ms]
 */
const int MAX_PT_DURATION = 2000;
/*
 * Maximum allowable go forwad manoeuver duration when commanded via TC [ms]
 */
const int MAX_GF_DURATION = 10000;
//------------------------------------------------------------------------------
// >> Internal Status variables
//------------------------------------------------------------------------------
/* 
 *  Broadcast distance sensor measurements to MQTT channel
 */
boolean isBroadcastDistance = false;

/*
 *  Broadcast full telemetry to Mqtt channel
 */
boolean enablePublishTelemetry = true;

/* 
 * Driving speed in auto mode [0 255] 
 */
int autoDrivingPace = 90;

/*
 * ControlDrive Point turn heading start val
 */
int CT_heading_start = 0;

/* 
 *  Time since radio is without contact
 */
long timeOffRadio = 0 ; 

int loop_counter_100Hz  = 0;
int loop_counter_10Hz   = 0;
// ================================================================
// ===                     [ STRUCT DEFS ]                      ===
// ================================================================
struct WheelStatus
{
  boolean isForward = false;
  boolean isStop    = true;
  int pace          = 0;
};

struct JoyData 
{
  int counter = 0 ;
  int X1      = 0 ;
  int Y1      = 0 ;
  int X2      = 0 ;
  int Y2      = 0 ;
};

struct Quat_float
{
  float w;
  float x;
  float y;
  float z;
};

struct Vec3_float
{
  float x;
  float y;
  float z;
};

struct YPR_float
{
  float yaw;
  float pitch;
  float roll;
};

struct ImuData
{
  
  int DELTA_TIME_THR = 100;
  unsigned long lastTimeTag_ms   = 0 ;

  int TIME_VALID    = 0 ;
  int TIME_INVALID  = 1;
  /*
   * If distance between two consecutive measurements is too large 
   * time based, computed measurements like position/velocity from 
   * acceleration are suppressed. 
   */
  int time_valid = TIME_INVALID;

  /*
   * Gravity Vector 
   * @Frame: IMU frame
   * @Unit: [m/s2]
   */
  Vec3_float vec_grav_ms2_IMU;
  
  /*
   * Accelerometer measurements [x,y,z] 
   * Note: Accelerometer measurements here are adjusted to 
   * remove the gravity vector
   * 
   * @Frame: IMU frame 
   * @Unit: [m/s2]
   */
  Vec3_float acc_ms2_IMU ;

  /*
   * Gyroscope measurements [gx, gy, gz] 
   * @Frame: IMU frame
   * @Unit: [rad/s]
   */
  Vec3_float gyro_rads2_IMU;
  /*
   * Temperature measurement
   * @Unit: [deg Celsius]
   */
  float temp_degC                   = 0 ;

  /*
   * Time delta wrt to previous IMU measurement 
   * @ Unit: [ms]
   */
  unsigned int delta_time_ms    = 0;

  /*
   * Quaternion describing rotations between frames:
   * @Frame: Global reference frame to IMU frame 
   * @Unit: [N/A]
   */
  Quat_float quat_Grf2Imu;
  
  /* 
   *  Direct cosine matrix Global reference frame to IMU frame 
   *  @Unit: [N/A]
   */
  float DCM_Grf2Imu[3][3] ;  

  /* 
   *  Direct cosine matrix Imu frame to Grf
   *  @Unit: [N/A]
   */
  float DCM_Imu2Grf[3][3] ; 
   
  /* 
   *  [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
   *  @Unit: [deg]
   */
  YPR_float euler_ypr_deg;  

 /*
  * @brief: Acceleration vector
  * @frame: Global Reference Frame
  * @Unit: meter/second^2
  */
  Vec3_float acc_ms2_GRF;

 /*
  * @brief: Velocity vector
  * @frame: Global Reference Frame
  * @Unit: meter/second
  */
  Vec3_float vel_m_GRF;
  
 /*
  * @brief: Position vector
  * @frame: Global Reference Frame
  * @Unit: meter
  */
  Vec3_float pos_m_GRF;
  
  int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
};

struct Telemetry 
{
  unsigned long clockTime      =  0 ;
  int drivingMode              =  0 ;
  int frontDistance            = -1 ;
  ImuData imuData;
  double busVoltage            =  0 ;
  
  /*
   * Internal status: Point turn execution status
   */
  int PT_IDLE       = 0 ;
  int PT_EXECUTING  = 1 ;
  int PT_COMPLETED  = 2 ;
  int pointTurnStatus =  0 ;
  WheelStatus statusWheel_A;
  WheelStatus statusWheel_B;
  WheelStatus statusWheel_C;
  WheelStatus statusWheel_D;
};

/*
 * Create global instance: telemetry
 */
Telemetry telemetry;

#endif
