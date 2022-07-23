#ifndef ROVER_SETTINGS_H

#define ROVER_SETTINGS_H

// ================================================================
// ===              [ VEHICLE MODES ]                           ===
// ================================================================
 const int IDLE_MODE            = 0;
 const int DEMO_SINGLE_WHEEL    = 1;
 const int DEMO_ALL_WHEEL       = 2;
 const int GAMEPAD_REMOTE_CTRL  = 3;
 const int DEMO_AUTO_DRIVE_1    = 4;
//-----------------------------------------------------------------
// !!! Set Drive Mode:
//-----------------------------------------------------------------
/*
 * Vehicle mode: 
 * 
 * @brief: Defines all functionality in loop 
 * 
 */
 int  VehicleMode = IDLE_MODE;


/*
 * Main loop Frequency [Hz]
 */
 int MAIN_LOOP_FREQUENCY_Hz = 100;

 /*
 * Telemetry loop Frequency [Hz]
 */
 int TM_LOOP_FREQUENCY_Hz    = 200;

/*
 * Publish frequency: Frequency at which full TM sets are published via MQTT
 */
 int TM_PUBLISH_FREQUENCY_HZ =  10;

 
 //------------------------------------------------------------------------------
 // Set telemetry announcements
 boolean tm_announce_drive_mode = true;

 boolean isSkipSensorInit = false;
 // Equipment
 boolean enableIMU        = true;
 boolean enableDistFront  = true;
 boolean enableRadio      = true;
 boolean enableWifi       = true;
 boolean enableMqtt       = true;
//------------------------------------------------------------------------------
// TM baud rate
const long tm_baud = 115200;

#endif
