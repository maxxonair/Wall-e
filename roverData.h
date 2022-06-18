
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

  //VectorFloat gravity;    // [x, y, z]            gravity vector
  //Quaternion q;           // [w, x, y, z]         quaternion container
  float euler[3];         // [psi, theta, phi]    Euler angle container
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

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
