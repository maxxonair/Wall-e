
#ifndef ROVER_MOTOR_CONTROL_H

#define ROVER_MOTOR_CONTROL_H

#include "roverData.h"
#include "roverPublish.h"
#include "roverImu.h"

//------------------------------------------------------------------------------
const int emptyPin = 34;
// Motor A
const int enA   = 33;
const int inA1 =  26;
const int inA2 =  27;
 
// Motor B
const int enB  =  25;
const int inB1 =  2;
const int inB2 =  15;

// Motor C
const int enC  =  13;
const int inC1 =  17;
const int inC2 =  16;

// Motor D
const int enD  =  32;
const int inD1 =  12;
const int inD2 =  14;

// Define Motor Telemetry
// Init status instance for each wheel
WheelStatus statusWheel_A;
WheelStatus statusWheel_B;
WheelStatus statusWheel_C;
WheelStatus statusWheel_D;

// Define Motor CMDs
const int CMD_FORWARD = 1;
const int CMD_REVERSE = 2;
const int CMD_STOP    = 0;
// Define Motor Speed Pulse Width Modulation Constants
const int PWMA_Ch  =  0;
const int PWMB_Ch  =  0;
const int PWMC_Ch  =  0;
const int PWMD_Ch  =  0;

const int PWM_Res  =  8;
const int PWM_Freq =  1000;

const int demoWheelSpeedSet = 90;
//------------------------------------------------------------------------------
void updateWheelStatus_A(boolean isForward, boolean isStop,  int pace){
  statusWheel_A.isForward     = isForward;
  statusWheel_A.isStop        = isStop;
  statusWheel_A.pace          = pace;
  telemetry.statusWheel_A     = statusWheel_A;
}

void updateWheelStatus_B(boolean isForward, boolean isStop,  int pace){
  statusWheel_B.isForward     = isForward;
  statusWheel_B.isStop        = isStop;
  statusWheel_B.pace          = pace;
  telemetry.statusWheel_B     = statusWheel_B;
}

void updateWheelStatus_C(boolean isForward, boolean isStop,  int pace){
  statusWheel_C.isForward     = isForward;
  statusWheel_C.isStop        = isStop;
  statusWheel_C.pace          = pace;
  telemetry.statusWheel_C     = statusWheel_C;
}

void updateWheelStatus_D(boolean isForward, boolean isStop,  int pace){
  statusWheel_D.isForward       = isForward;
  statusWheel_D.isStop          = isStop;
  statusWheel_D.pace            = pace;
  telemetry.statusWheel_D       = statusWheel_D;
}

void ctrlWheel_A(int cmd, int wheelSpeed){
  // Set speed 
  ledcWrite(PWMA_Ch, wheelSpeed);
  // Set wheel speed direction
  switch (cmd) {
    
    case WHEEL_DRIVE_FORWARD:
      digitalWrite(inA1, HIGH);
      digitalWrite(inA2, LOW);
      break;
    case WHEEL_DRIVE_REVERSE:
      digitalWrite(inA1, LOW);
      digitalWrite(inA2, HIGH);
      break;
    case WHEEL_DRIVE_STOP:
      digitalWrite(inA1, HIGH);
      digitalWrite(inA2, HIGH);
      break;
    default:
      // Default: Stop
      digitalWrite(inA1, HIGH);
      digitalWrite(inA2, HIGH);
      break;
      
  }
  boolean isStop    = false;
  boolean isForward = false;
  if ( cmd == CMD_FORWARD ) 
  {
    isForward = true;
  } else if ( cmd == CMD_STOP ) 
  {
    isStop     = true;
    wheelSpeed = 0;
  } 
  updateWheelStatus_A(isForward, isStop, wheelSpeed);
}

void ctrlWheel_B(int cmd, int wheelSpeed){

  // Set speed out of possible range 0~255 
  ledcWrite(PWMB_Ch, wheelSpeed);
  // Set wheel speed direction
  switch (cmd) {
    
    case WHEEL_DRIVE_FORWARD:
      digitalWrite(inB1, HIGH);
      digitalWrite(inB2, LOW);
      break;
    case WHEEL_DRIVE_REVERSE:
      digitalWrite(inB1, LOW);
      digitalWrite(inB2, HIGH);
      break;
    case WHEEL_DRIVE_STOP:
      digitalWrite(inB1, HIGH);
      digitalWrite(inB2, HIGH);
      break;
    default:
      // Default: Stop
      digitalWrite(inB1, HIGH);
      digitalWrite(inB2, HIGH);
      break;
      
  }
  boolean isStop    = false;
  boolean isForward = false;
  if ( cmd == CMD_FORWARD ) 
  {
    isForward = true;
  } else if ( cmd == CMD_STOP ) 
  {
    isStop     = true;
    wheelSpeed = 0;
  }
  updateWheelStatus_B(isForward, isStop, wheelSpeed);
}

void ctrlWheel_C(int cmd, int wheelSpeed){

  // Set speed out of possible range 0~255 
  ledcWrite(PWMC_Ch, wheelSpeed);
  // Set wheel speed direction
  switch (cmd) {
    
    case WHEEL_DRIVE_FORWARD:
      digitalWrite(inC1, HIGH);
      digitalWrite(inC2, LOW);
      break;
    case WHEEL_DRIVE_REVERSE:
      digitalWrite(inC1, LOW);
      digitalWrite(inC2, HIGH);
      break;
    case WHEEL_DRIVE_STOP:
      digitalWrite(inC1, HIGH);
      digitalWrite(inC2, HIGH);
      break;
    default:
      // Default: Stop
      digitalWrite(inC1, HIGH);
      digitalWrite(inC2, HIGH);
      break;
      
  }
  boolean isStop    = false;
  boolean isForward = false;
  if ( cmd == CMD_FORWARD ) 
  {
    isForward = true;
  } else if ( cmd == CMD_STOP ) 
  {
    isStop     = true;
    wheelSpeed = 0;
  }
  updateWheelStatus_C(isForward, isStop, wheelSpeed);
}

void ctrlWheel_D(int cmd, int wheelSpeed){

  // Set speed out of possible range 0~255 
  ledcWrite(PWMD_Ch, wheelSpeed);
  // Set wheel speed direction
  switch (cmd) {
    
    case WHEEL_DRIVE_FORWARD:
      digitalWrite(inD1, HIGH);
      digitalWrite(inD2, LOW);
      break;
    case WHEEL_DRIVE_REVERSE:
      digitalWrite(inD1, LOW);
      digitalWrite(inD2, HIGH);
      break;
    case WHEEL_DRIVE_STOP:
      digitalWrite(inD1, HIGH);
      digitalWrite(inD2, HIGH);
      break;
    default:
      // Default: Stop
      digitalWrite(inD1, HIGH);
      digitalWrite(inD2, HIGH);
      break;
      
  }
  boolean isStop    = false;
  boolean isForward = false;
  if ( cmd == CMD_FORWARD ) 
  {
    isForward = true;
  } else if ( cmd == CMD_STOP ) 
  {
    isStop     = true;
    wheelSpeed = 0;
  }
  updateWheelStatus_D(isForward, isStop, wheelSpeed);
}

void ctrlAllWheel_Stop(){
        ctrlWheel_A(CMD_STOP , 0);
        ctrlWheel_B(CMD_STOP , 0);
        ctrlWheel_C(CMD_STOP , 0);
        ctrlWheel_D(CMD_STOP , 0);
}

void ctrlAllWheel_Forward( int pace ){
    ctrlWheel_A(CMD_FORWARD , pace);
    ctrlWheel_B(CMD_FORWARD , pace);
    ctrlWheel_C(CMD_FORWARD , pace);
    ctrlWheel_D(CMD_FORWARD , pace);
}

void ctrlAllWheel_Reverse( int pace ){
    ctrlWheel_A(CMD_REVERSE , pace);
    ctrlWheel_B(CMD_REVERSE , pace);
    ctrlWheel_C(CMD_REVERSE , pace);
    ctrlWheel_D(CMD_REVERSE , pace);
}

void ctrlAllWheel_RotateRightward(int pace){
      ctrlWheel_A(CMD_FORWARD , pace);
      ctrlWheel_D(CMD_FORWARD , pace);
      ctrlWheel_C(CMD_REVERSE , pace);
      ctrlWheel_B(CMD_REVERSE , pace);
}

void ctrlAllWheel_RotateLeftward(int pace){
      ctrlWheel_A(CMD_REVERSE , pace);
      ctrlWheel_D(CMD_REVERSE , pace);
      ctrlWheel_C(CMD_FORWARD , pace);
      ctrlWheel_B(CMD_FORWARD , pace);
}

void singleWheelDemo(){
  // <Alternating single wheel (forward/backward) sequence to check wiring>
  // Wheel (A) Demo 
  Serial.println("Wheel A");
  delay(800);
  ctrlWheel_A(CMD_FORWARD , demoWheelSpeedSet);
  delay(1000);
  ctrlWheel_A(CMD_STOP    ,   0);
  delay(200);
  ctrlWheel_A(CMD_REVERSE , demoWheelSpeedSet);
  delay(1000);
  ctrlWheel_A(CMD_STOP    ,   0);

  // Wheel (B) Demo 
  Serial.println("Wheel B");
  delay(800);
  ctrlWheel_B(CMD_FORWARD , demoWheelSpeedSet);
  delay(1000);
  ctrlWheel_B(CMD_STOP    ,   0);
  delay(200);
  ctrlWheel_B(CMD_REVERSE , demoWheelSpeedSet);
  delay(1000);
  ctrlWheel_B(CMD_STOP    ,   0);

  // Wheel (C) Demo 
  Serial.println("Wheel C");
  delay(800);
  ctrlWheel_C(CMD_FORWARD , demoWheelSpeedSet);
  delay(1000);
  ctrlWheel_C(CMD_STOP    ,   0);
  delay(200);
  ctrlWheel_C(CMD_REVERSE , demoWheelSpeedSet);
  delay(1000);
  ctrlWheel_C(CMD_STOP    ,   0);
  
  // Wheel (D) Demo 
  Serial.println("Wheel D");
  delay(800);
  ctrlWheel_D(CMD_FORWARD , demoWheelSpeedSet);
  delay(1000);
  ctrlWheel_D(CMD_STOP    ,   0);
  delay(200);
  ctrlWheel_D(CMD_REVERSE , demoWheelSpeedSet);
  delay(1000);
  ctrlWheel_D(CMD_STOP    ,   0);
}

void demoAllWheels(){
   // Wheel (A) Demo 
  ctrlWheel_A(CMD_FORWARD , 255);
  ctrlWheel_B(CMD_FORWARD , 255);
  ctrlWheel_C(CMD_FORWARD , 255);
  ctrlWheel_D(CMD_FORWARD , 255);
  delay(1000);
  ctrlWheel_A(CMD_STOP , 255);
  ctrlWheel_B(CMD_STOP , 255);
  ctrlWheel_C(CMD_STOP , 255);
  ctrlWheel_D(CMD_STOP , 255);
  delay(1000);
  ctrlWheel_A(CMD_REVERSE , 255);
  ctrlWheel_B(CMD_REVERSE , 255);
  ctrlWheel_C(CMD_REVERSE , 255);
  ctrlWheel_D(CMD_REVERSE , 255);
  delay(1000);
  ctrlWheel_A(CMD_STOP , 255);
  ctrlWheel_B(CMD_STOP , 255);
  ctrlWheel_C(CMD_STOP , 255);
  ctrlWheel_D(CMD_STOP , 255);
   delay(1000);
}

void shake(float frequency, float t){
  // period in s
  float period = 1/frequency;
  
  // Number of cycles
  int cycles = (int) (t/period);
  
    for (int ii = 1; ii < cycles ; ii++){   
  
    ctrlAllWheel_Forward(150);
    delay(period/3*1000);
    ctrlAllWheel_Stop();
    delay(period/3*1000);
    ctrlAllWheel_Reverse(150);;
    delay(period/3*1000);
    
  }

}

void comeToLifeRoutine(){
  // Shake 30 Hz for 2 seconds
  shake(40,1.5);
}

/*
 * Execute point turn with random angle and direction
 */
void performRandomPointTurn( ){
    telemetry.pointTurnStatus = telemetry.PT_EXECUTING;
    // Publish maneuver 
    publishData( String("[Wall-e][TM] Init point turn. "), MQTT_TOPIC_STATUS);
    
    // Perform turn maneuver:
    ctrlAllWheel_Stop();
    ctrlAllWheel_Reverse(autoDrivingPace);
    delay(700);
    if (random(0,1000) > 230) {
      ctrlAllWheel_RotateRightward(autoDrivingPace);
    } else {
      ctrlAllWheel_RotateLeftward(autoDrivingPace);
    }
    delay(random(350,650));
    ctrlAllWheel_Stop();
    delay(20);
    telemetry.pointTurnStatus = telemetry.PT_COMPLETED;
}

/*
 * Perform turn for turn_dur_ms milli seconds
 * Pace: autoDrivingPace
 */
void performPointTurn( int directionIndex, int turn_dur_ms ){
    telemetry.pointTurnStatus = telemetry.PT_EXECUTING;
    // Publish maneuver 
    publishData( String("[Wall-e][TM] Init point turn. "), MQTT_TOPIC_STATUS);
    
    /*
     * All wheel stop
     */
    ctrlAllWheel_Stop();
    /*
     * Rotate counterclockwise 
     */
     if ( directionIndex == 0 )
     {
      ctrlAllWheel_RotateRightward(autoDrivingPace);
     } 
     else if ( directionIndex == 1 )
     {
      ctrlAllWheel_RotateLeftward(autoDrivingPace);
     }
    
    delay(turn_dur_ms);
    /*
     * All Wheels Stop
     */
    ctrlAllWheel_Stop();
    
    delay(20);
    telemetry.pointTurnStatus = telemetry.PT_COMPLETED;
}

/*
 * Command manoevuer go forward in straight line for go_dur_ms [ms]
 * Pace: autoDrivingPace
 */
void performGoForward( int go_dur_ms ){
    
    /*
     * All wheel stop
     */
    ctrlAllWheel_Stop();
    /*
     * Rotate counterclockwise 
     */
    ctrlAllWheel_Forward( autoDrivingPace );
    delay(go_dur_ms);
    /*
     * All Wheels Stop
     */
    ctrlAllWheel_Stop();
    
    delay(20);
}

/*
 * Command manoevuer go reverse in straight line for go_dur_ms [ms]
 * Pace: autoDrivingPace
 */
void performGoReverse( int go_dur_ms ){
    
    /*
     * All wheel stop
     */
    ctrlAllWheel_Stop();
    /*
     * Rotate counterclockwise 
     */
    ctrlAllWheel_Reverse( autoDrivingPace );
    delay(go_dur_ms);
    /*
     * All Wheels Stop
     */
    ctrlAllWheel_Stop();
    
    delay(20);
}

/*
 * Command manoevuer go point turn in ControlDrive mode for delta_rotation [deg]
 * Pace: controller commanded pace 
 */
int performCtPointTurn( int delta_rotation ){
    int ctrl_error = 0;
    int abs_ctrl_error=0;
    int delay_dt = 100;
    int turn_pace_cmd ;
    /* Target accuracy treshold [deg] */
    int target_thr      =   4;
    int max_attempts    = 100;
    int attempt_counter =   0;
    /* assign start heading */
    CT_heading_start = imuData.euler_ypr_deg.yaw;
    /*
     * All wheel stop
     */
    ctrlAllWheel_Stop();
    /* Reset heading controller */
    ctrl_reset(heading_ctrl);
    /* Set start ctrl error */
    abs_ctrl_error = ( imuData.euler_ypr_deg.yaw - delta_rotation );
    
    while ( abs_ctrl_error > target_thr && attempt_counter < max_attempts )
    {
      /*
       * Rotate clockwise 
       */
      /* compute controller error */
      ctrl_error = ( imuData.euler_ypr_deg.yaw - delta_rotation );
      abs_ctrl_error = abs( ctrl_error );
      
      turn_pace_cmd = int( ctrl_update( abs_ctrl_error, (delay_dt/1000), heading_ctrl) );

      if ( ctrl_error > 0 ){
        ctrlAllWheel_RotateLeftward(turn_pace_cmd); 
      } 
      else 
      {
        ctrlAllWheel_RotateRightward(turn_pace_cmd);
      }
      

      attempt_counter = attempt_counter +1;
      delay(delay_dt);
    }


    /*
     * All Wheels Stop
     */
    ctrlAllWheel_Stop();
    
    delay(20);

    if (ctrl_error > target_thr)
    {
      return 0;
    } else 
    {
      return 1;
    }
}

#endif
