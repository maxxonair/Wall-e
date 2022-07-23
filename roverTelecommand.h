
#ifndef ROVER_TELECOMMAND_H

#define ROVER_TELECOMMAND_H

#include "StringSplitter.h"

#include "roverData.h"
#include "roverPublish.h"
#include "roverMotorCntrl.h"


/*
 * Function: Process Telecommand -> Set Parameter to Value 
 */
void processTelecommand( String Parameter, String Value ){
 if ( Parameter == TC_SET_VEHICLE_MODE )
  {
    int opsModeInt = Value.toInt();
    if ( opsModeInt == IDLE_MODE || opsModeInt == DEMO_SINGLE_WHEEL || opsModeInt == DEMO_ALL_WHEEL || opsModeInt == GAMEPAD_REMOTE_CTRL || opsModeInt == DEMO_AUTO_DRIVE_1)
    {
      VehicleMode = opsModeInt;
      publishData( String("[Wall-e][TM] Setting Operational Mode: "+Value), MQTT_TOPIC_STATUS );
    } 
    else
    {
      publishData( String("[Wall-e][TM] Operational Mode not valid. Rejecting Command. "), MQTT_TOPIC_STATUS );
    }
  }
  else if ( Parameter == TC_BROADCAST_DIST )
  {
    int distModeInt = Value.toInt();
    if ( distModeInt != 0 )
    {
      isBroadcastDistance = true;
      publishData( String("[Wall-e][TM] Setting Broadcast Distance: Enabled"), MQTT_TOPIC_STATUS );
    } 
    else
    {
      isBroadcastDistance = false;
      publishData( String("[Wall-e][TM] Setting Broadcast Distance: Disabled"), MQTT_TOPIC_STATUS );
    }
  }
  else if ( Parameter == TC_SET_AUTO_PACE )
  {
    int autoPaceInt = Value.toInt();
    if ( autoPaceInt > 0 && autoPaceInt <= 255 )
    {
      autoDrivingPace = autoPaceInt;
      publishData( String("[Wall-e][TM] Setting Auto Drive Pace: "+Value), MQTT_TOPIC_STATUS );
    } 
    else
    {
      publishData( String("[Wall-e][TM] Auto Drive Pace command not valid. Rejecting Command. "), MQTT_TOPIC_STATUS );
    }
  }
  else if ( Parameter == TC_ENABLE_TM )
  {
    int tmInt = Value.toInt();
    if ( tmInt == 0 || tmInt == 1 )
    {
      enablePublishTelemetry = tmInt;
      publishData( String("[Wall-e][TM] Setting Enable Telemetry broadcast: "+Value), MQTT_TOPIC_STATUS );
    } 
  }
  else if ( Parameter == TC_POINT_TURN_0 )
  {
    int pt_duration_ms = Value.toInt();
    if ( pt_duration_ms > 0 || pt_duration_ms < MAX_PT_DURATION )
    {
      publishData( String("[Wall-e][TM] Perform point turn clockwise duration: "+Value+" [ms]"), MQTT_TOPIC_STATUS );
      performPointTurn(0 ,pt_duration_ms);
    } 
    else
    {
      publishData( String("[Wall-e][TM] Point turn duration outside limits. Rejecting"), MQTT_TOPIC_STATUS );
    }
  }
  else if ( Parameter == TC_POINT_TURN_1 )
  {
    int pt_duration_ms = Value.toInt();
    if ( pt_duration_ms > 0 || pt_duration_ms < MAX_PT_DURATION )
    {
      publishData( String("[Wall-e][TM] Perform point turn counter-clockwise duration: "+Value+" [ms]"), MQTT_TOPIC_STATUS );
      performPointTurn( 1, pt_duration_ms);
    } 
    else
    {
      publishData( String("[Wall-e][TM] Point turn duration outside limits. Rejecting"), MQTT_TOPIC_STATUS );
    }
  }
  else if ( Parameter == TC_GO_FORWARD )
  {
    int go_forward_ms = Value.toInt();
    if ( go_forward_ms > 0 || go_forward_ms < MAX_GF_DURATION )
    {
      publishData( String("[Wall-e][TM] Go forward for: "+Value+" [ms]"), MQTT_TOPIC_STATUS );
      performGoForward(go_forward_ms);
    } 
    else
    {
      publishData( String("[Wall-e][TM] Go forward duration outside limits. Rejecting"), MQTT_TOPIC_STATUS );
    }
  }
  else if ( Parameter == TC_GO_REVERSE )
  {
    int go_reverse_ms = Value.toInt();
    if ( go_reverse_ms > 0 || go_reverse_ms < MAX_GF_DURATION )
    {
      publishData( String("[Wall-e][TM] Go reverse for: "+Value+" [ms]"), MQTT_TOPIC_STATUS );
      performGoReverse(go_reverse_ms);
    } 
    else
    {
      publishData( String("[Wall-e][TM] Go reverse duration outside limits. Rejecting"), MQTT_TOPIC_STATUS );
    }
  }
  else if ( Parameter == TC_SET_PID_P )
  {
    float PID_val = Value.toFloat();
    if ( PID_val >= 0  )
    {
      publishData( String("[Wall-e][TM] Set PID P gain: "+Value+" [-]"), MQTT_TOPIC_STATUS );
      heading_ctrl.P_GAIN = PID_val;
    } 
    else
    {
      publishData( String("[Wall-e][TM] PID gain command outside limits. Rejecting"), MQTT_TOPIC_STATUS );
    }
  }
  else if ( Parameter == TC_SET_PID_I )
  {
    float PID_val = Value.toFloat();
    if ( PID_val >= 0  )
    {
      publishData( String("[Wall-e][TM] Set PID I gain: "+Value+" [-]"), MQTT_TOPIC_STATUS );
      heading_ctrl.I_GAIN = PID_val;
    } 
    else
    {
      publishData( String("[Wall-e][TM] PID gain command outside limits. Rejecting"), MQTT_TOPIC_STATUS );
    }
  }
  else if ( Parameter == TC_SET_PID_D )
  {
    float PID_val = Value.toFloat();
    if ( PID_val >= 0  )
    {
      publishData( String("[Wall-e][TM] Set PID D gain: "+Value+" [-]"), MQTT_TOPIC_STATUS );
      heading_ctrl.D_GAIN = PID_val;
    } 
    else
    {
      publishData( String("[Wall-e][TM] PID gain command outside limits. Rejecting"), MQTT_TOPIC_STATUS );
    }
  }
  else if ( Parameter == TC_GO_POINT_TURN_CT )
  {
    int rotAngle = Value.toInt();
    if ( abs(rotAngle) <= 180  )
    {
      publishData( String("[Wall-e][TM][ControlDrive] Perform point turn counter-clockwise. Rotation for: "+Value+" [deg]"), MQTT_TOPIC_STATUS );
      performCtPointTurn( rotAngle );
    } 
    else
    {
      publishData( String("[Wall-e][TM][ControlDrive] Point turn command outside limits. Rejecting"), MQTT_TOPIC_STATUS );
    }
  }
  else
  {
    publishData( String("[Wall-e][TM] TC not valid. Rejecting. "), MQTT_TOPIC_STATUS );
  }

}

/*
 * Function: Parse and process Telecommand 
 */
void parseAndProcessTelecommand(String Telecommand){
  StringSplitter *splitter = new StringSplitter(Telecommand, ';', 3);
  int itemCount = splitter->getItemCount();
  int success_flag      =  0 ;
  String tc_iden        = "" ;
  String parameter_iden = "" ;
  String value_iden     = "" ;

  if ( itemCount != 3 ) {
      /*
       * Add message here 
       */
       publishData( "[Wall-e][TM] TC parsing failed. Rejecting TC.", MQTT_TOPIC_STATUS );
  } else {
      /*
       * Parse TC 
       */
      for(int i = 0; i < itemCount; i++){
        if ( i == 0 ){
          tc_iden = splitter->getItemAtIndex(i);
        } else if ( i == 1 ) {
          parameter_iden = splitter->getItemAtIndex(i);
        } else if ( i == 2 ) {
          value_iden = splitter->getItemAtIndex(i);
        }
      }
    
      /*
       * Process TC 
       */
      if ( tc_iden != TC_IDENTIFIER ){
        publishData( "[Wall-e][TM] Wrong TC identifier. Rejecting TC.", MQTT_TOPIC_STATUS );
      } else {
        //publishTcAckn(parameter_iden, value_iden);
        processTelecommand(parameter_iden, value_iden);
      }
  } 
}
#endif
 
