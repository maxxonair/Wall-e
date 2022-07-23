//--------------------------------------------------------------------------------------------------------
//           				                PID controller 
//  -> This class contains a set of PID controller to be used in any abitrary setup as flight controller
// 
//
//
//--------------------------------------------------------------------------------------------------------

#ifndef ROVER_PID__H

#define ROVER_PID__H
/*
 *   PID controller structure 
 */
struct PidData 
{
  /*
  * Controller gains
  */
  double P_GAIN ;
  double I_GAIN ;
  double D_GAIN ;
  /*
   * Controller command boundaries 
   */
  double CMD_MAX;
  double CMD_MIN;
  int enable_boundary_control;

  double last_error;
  double error_integral;

};

/*
 * Function: Reset controller state 
 */
void ctrl_reset(PidData PID_state_in_out) 
{
    PID_state_in_out.error_integral = 0 ;
    PID_state_in_out.last_error     = 0 ;
}

/*
 * Function: Compute PID controllor update state and compute controller command
 */
double ctrl_update(double ctrl_error_in, double dt_s, PidData PID_state_in_out){
    //--------------------------------------------------------------------------------------------------------
    //           				Constrained PID controller 
    //  -> Given maximum and minimum constraints can not be exceeded by the controller output command 
    //--------------------------------------------------------------------------------------------------------
    double ACT_CMD=0;							// Actuator Command
    double P_CMD=0;								// Proportional tuning command
    double I_CMD=0;								// Integral tuning command
    double D_CMD=0;								// Derivative tuning command 

    P_CMD = PID_state_in_out.P_GAIN * ctrl_error_in;
    PID_state_in_out.error_integral = PID_state_in_out.error_integral + (ctrl_error_in * dt_s);
    I_CMD = PID_state_in_out.I_GAIN*(PID_state_in_out.error_integral);
    if( dt_s > 1e-6 ) 
    {
        D_CMD = PID_state_in_out.D_GAIN*(ctrl_error_in-PID_state_in_out.last_error)/dt_s;} else {
        D_CMD = 0;
    }

    ACT_CMD = P_CMD + I_CMD + D_CMD;

    /*
     * Check controller boundaries and restrict output commands 
     */
    if ( PID_state_in_out.enable_boundary_control == 1 ) 
    {
        if (ACT_CMD>PID_state_in_out.CMD_MAX)       
        {
            ACT_CMD = PID_state_in_out.CMD_MAX;				
        } 
        else if (ACT_CMD<PID_state_in_out.CMD_MIN) 
        {
            ACT_CMD = PID_state_in_out.CMD_MIN;				
        } 
    }

    PID_state_in_out.last_error = ctrl_error_in; 
    return ACT_CMD;
}

#endif