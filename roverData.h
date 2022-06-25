#ifndef ROVER_DATA_H

#define ROVER_DATA_H

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

struct ImuData
{
  unsigned long timeTag   = 0 ;
  double ax               = 0 ;
  double ay               = 0 ;
  double az               = 0 ;
  double gx               = 0 ;
  double gy               = 0 ; 
  double gz               = 0 ;
  double temperature      = 0 ;

  /*
   * Time delta wrt to previous IMU measurement [ms]
   */
  unsigned long delta_time_ms    = 0;
  
  double delta_axis_angle_rad[3];

  double DCM_Grf2Vbf[3][3] ;       // Direct cosine matrix Global reference frame to vehicle body frame 
  double euler_ypr[3];             // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

  int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;
};

struct Telemetry 
{
  unsigned long clockTime      =  0 ;
  int drivingMode     =  0 ;
  int frontDistance   = -1 ;
  ImuData imuData;
  double busVoltage   =  0 ;
  
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

#endif
