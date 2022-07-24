//------------------------------------------------------------------------------
/*
 * 
 *          Wall-e Firmware
 *          
 *      RoverOne Debug & Drive   -- mk 1.1
 *      
 *      Hardware: 
 *      OBC:              ESP32 Dev Kit
 *      IMU:              MPU6050
 *      Front Distance:   VL53L0X
 *      Additional Radio: NRF24L01
 *      Motor Control:    2 x L297
 *      Power:            3 x 18650 cells (3S)
 *      
 *      Payload:          ESP32 AI Thinker Camera (@ http://192.168.1.191/ )
 * 
 *         ----------- 
 *         |         |
 * Wheel A |A       C| Wheel C
 *         |         |
 * Wheel D |D       B| Wheel B
 *         |    v    |
 *         -----------
 *        
 *        Radio Receiver Setup:
 *        
 *        Module: NRF24L01 Radio Transmitter/Receiver
 *        Pins:
 *                SCK  -> 18
 *                MISO -> 19
 *                MOSI -> 23
 *                CSN  ->  5
 *                CE   ->  4
 *        
 * 
 * @dependencies:
 * - StringSplitter
 * - Pubsubclient
 */
//------------------------------------------------------------------------------
//                                Libraries
//------------------------------------------------------------------------------
// WIFI
#include <WiFi.h>
#include "ESP32_PubSubClient.h"
// nRF24L01 Radio
#include "RF24.h"
#include <nRF24L01.h>
#include <SPI.h>
// Time of Flight Sensor 
#include "Adafruit_VL53L0X.h"
// IMU MPU6050
#include <Adafruit_Sensor.h>

// MQTT
#include "Esp32MQTTClient.h"

// Include [rover] files 
#include "roverTelemetry.h"
#include "wifiAccessCodes.h"
#include "roverMqtt.h"
#include "roverData.h" 
#include "roverImu.h"
#include "roverMotorCntrl.h"
#include "roverJoystickCntrl.h"
#include "roverSensorFunction.h"
#include "roverRadio.h"
#include "roverLaserDistance.h"
#include "roverPid.h"

void manageTelemetry( void * pvParameters );
// ================================================================
// ===                    SETUP                                 ===
// ================================================================
void setup() {
  // Setup TM serial
  Serial.begin(tm_baud);
  delay(10);
    
  /* 
   *  Set all the motor control pins to outputs
   */
  pinMode(inA1, OUTPUT);
  pinMode(inA2, OUTPUT);
  
  pinMode(inB1, OUTPUT);
  pinMode(inB2, OUTPUT);
  
  pinMode(inC1, OUTPUT);
  pinMode(inC2, OUTPUT);
  
  pinMode(inD1, OUTPUT);
  pinMode(inD2, OUTPUT);
  
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(enC, OUTPUT);
  pinMode(enD, OUTPUT);

  ledcSetup(PWMA_Ch, PWM_Freq, PWM_Res);
  ledcAttachPin(enA, PWMA_Ch);

  ledcSetup(PWMB_Ch, PWM_Freq, PWM_Res);
  ledcAttachPin(enB, PWMB_Ch);

  ledcSetup(PWMC_Ch, PWM_Freq, PWM_Res);
  ledcAttachPin(enC, PWMC_Ch);

  ledcSetup(PWMD_Ch, PWM_Freq, PWM_Res);
  ledcAttachPin(enD, PWMD_Ch);

  /*
   * [SETUP HEADING PID CTRL]
   */
   setupHeadingController();

  /*
   * [ INIT WIFI ]
   */
  if ( enableWifi ) {
    wifiConnection_init();
  }

  /* 
   *  [ Init MQTT ]
   */
  if ( enableMqtt) 
  {
    mqttConnection_init();
  }
  
  publishData(String("[Wall-e][TM] Switching ON."), MQTT_TOPIC_STATUS);
  publishData(String("[Wall-e][TM][INIT] Wifi and MQTT connceted."), MQTT_TOPIC_STATUS);
  publishData(String("[Wall-e][TM][INIT] Start module initialization:"), MQTT_TOPIC_STATUS);
  
  /* 
   *  INIT MPU-6050 Inertial Measurement Unit
   */
  if ( enableIMU && !isSkipSensorInit )
  {
    publishData(String("[Wall-e][TM][IMU] IMU Initialization. Don't Move the rover."), MQTT_TOPIC_STATUS);
    if ( initIMU() ){
      
      publishData(String("[Wall-e][TM][IMU] IMU Initialization failed."), MQTT_TOPIC_STATUS);
      
    } else {
      
      publishData(String("[Wall-e][TM][IMU] IMU Initialization and Calibration completed."), MQTT_TOPIC_STATUS);
      
    }

    delay(10);

  }


  // INIT VL53L0X 
  if (enableDistFront && !isSkipSensorInit){
      // all reset
    publishData(String("[Wall-e][TM][VL53L0X] VL53L0X Front Initialization."), MQTT_TOPIC_STATUS);
    digitalWrite(SHUT_VL, LOW);    
    delay(10);
    // all unreset
    digitalWrite(SHUT_VL, HIGH);
    delay(10);
    if (!vl53lox.begin(LOX1_ADDRESS)) {
      publishData(String("[Wall-e][TM][VL53L0X] Failed to initialize."), MQTT_TOPIC_STATUS);
    }
    Serial.println(">> VL53L0X Sensor Online."); 
  } else if (enableDistFront) {
    vl53lox.begin(LOX1_ADDRESS);
  }
 
  // INIT NRF24
  if (enableRadio) {
    Serial.println("INIT: NRF24");
    publishData(String("[Wall-e][TM][NRF24] Initialize NRF24 radio transmitter/receiver."), MQTT_TOPIC_STATUS);
    radio.begin();
    
    //set the address
    radio.openReadingPipe(0, NRF24_PIPE_ADR);
    
    //Set module as receiver
    radio.startListening();
  }

  /*
   * From here onwards publishData can be used
   */
  publishData(String("[Wall-e][TM] Execute come to life routine."), MQTT_TOPIC_STATUS);

  /*
   * Setup Telemetry on the seconds core 
   */
   publishData(String("[Wall-e][TM][INIT] Setup telemetry on second core."), MQTT_TOPIC_STATUS);
   xTaskCreatePinnedToCore(
                    manageTelemetry,        /* Task function. */
                    "manageTelemetry",      /* name of task. */
                    10000,                  /* Stack size of task */
                    NULL,                   /* parameter of the task */
                    1,                      /* priority of the task */
                    &TelemetryTask,         /* Task handle to keep track of created task */
                    TM_TASK_AT_CORE_ID);    /* pin task to core TM_TASK_AT_CORE_ID */                  
  // Let that settle ...
  delay(50);
  
  // Come to life routine
  publishData(String("[Wall-e][TM][INIT] Execute come to life routine."), MQTT_TOPIC_STATUS);
  /*
   * Quick shake to signal:
   * (a) Automotive functions are operational  
   * (b) Intialization has been completed successfully
   * 
   */
  comeToLifeRoutine();
  
  /* 
   * Set all wheels >> STOP 
   */
  ctrlAllWheel_Stop();  

  /*
   * Reset timer off-radio connection
   */
  long timeOffRadio = 0 ; 

  publishData(String("[Wall-e][TM][INIT] > Initialization complete <"), MQTT_TOPIC_STATUS);
}

// ==============================================================================
// ===                          MAIN PROGRAM LOOP                             ===
// ==============================================================================
void loop() {
//-------------------------------------------------------------------------------
  if ( enableMqtt )
  {
    /* 
     *  If not connected to broker -> connect
     */
    if (!client.connected() && WiFi.status() == WL_CONNECTED ) 
    {
      if ( connectMqtt() == 0 ) 
      {
        connection_failures = connection_failures + 1;
        broker_connection_failures = broker_connection_failures + 1;
      }
    } 
  }
  
//-------------------------------------------------------------------------------  
  switch (VehicleMode) {

    /*
     * Idle Mode: All wheels stop
     * If Radio signal available -> switch to GAMEPAD_REMOTE_CTRL
     */
    case IDLE_MODE:
      // Stay idle 
      if( radio.available() )  {
          setDrivingMode( GAMEPAD_REMOTE_CTRL );
      }
      ctrlAllWheel_Stop();
      break;

    /*
     * Demo: Alternating single wheel 
     */
    case DEMO_SINGLE_WHEEL:
        singleWheelDemo();
        break;

    /*
     * Demo: all wheels forward
     */
    case DEMO_ALL_WHEEL:
         demoAllWheels();
         break;

    /*
     * Rover Remote Controlled via radio gamepad
     */
    case GAMEPAD_REMOTE_CTRL:
      // Remote Joystick Control only:
      //ctrlAllWheel_Stop();
      if( radio.available() )  {
        timeOffRadio = millis() ; 
        enableJoystickCtrl(radio);
      } else {
        int switchThrTime = 2500;
        if ( (millis() - timeOffRadio) > switchThrTime ){
          setDrivingMode( IDLE_MODE );
          break;
        } 
      }
      break;

    /*
     * Auto Drive 1 : Obstacle avoidance using front laser distnace sensor (triggering point turns)
     */
    case DEMO_AUTO_DRIVE_1:
        // Automatic drive demo with object detection 
        // Object detection with time of flight sensor @ front/center       
        if( radio.available() )  {
            /*
             * If Radio signal available switch vehicle mode to remote control 
             */
            setDrivingMode( GAMEPAD_REMOTE_CTRL );
            break;
        } else  {

            /*
             * PT init thresholds hard coded here for now since its 
             * easier to play around with them :) 
             */
            int turnDist   = 250;
            double turnVel = 0.05;
            
            if ( telemetry.frontDistance < turnDist && telemetry.frontDistance != -1 ) //|| abs(fVelocity) < turnVel && fVelocity != -1 ) 
            {
                performRandomPointTurn();              
            } else {
              ctrlAllWheel_Forward( autoDrivingPace );
            }
        }

        break;
    default:
        ctrlAllWheel_Stop();
        break;
  }

/*
 * Loop Mqtt 
 */
client.loop();

/*
 * Main loop delay
 */
delay(int( 1/MAIN_LOOP_FREQUENCY_Hz * 1000 ));
}

/*
 * Function: Set Vehicle mode 
 */
void setDrivingMode(int newMode){
  VehicleMode           = newMode;
  telemetry.drivingMode = newMode;
  publishData( String("[Wall-e][TM] Change Vehicle Mode to: ")+String(newMode), MQTT_TOPIC_STATUS);
}


//-------------------------------------------------------------------------

/*
 * Function Serialize Telemetry structure to csv row
 */
String serializeTelemetry(){
  String TM = "";
  String dm = ";";
  TM = "[TM]"
  +dm+String(telemetry.clockTime)+dm+String(telemetry.drivingMode)+dm+String(telemetry.pointTurnStatus)
  +dm+String(telemetry.frontDistance)+dm+String(telemetry.imuData.temp_degC)
  +dm+String(telemetry.imuData.acc_ms2_IMU.x)+dm+String(telemetry.imuData.acc_ms2_IMU.y)+dm+String(telemetry.imuData.acc_ms2_IMU.z)
  +dm+String(telemetry.imuData.quat_Grf2Imu.w)+dm+String(telemetry.imuData.quat_Grf2Imu.x)+dm+String(telemetry.imuData.quat_Grf2Imu.y)+dm+String(telemetry.imuData.quat_Grf2Imu.z)
  +dm+String(telemetry.imuData.euler_ypr_deg.yaw,3)+dm+String(telemetry.imuData.euler_ypr_deg.pitch)+dm+String(telemetry.imuData.euler_ypr_deg.roll)
  +dm+String(telemetry.imuData.pos_m_GRF.x,3)+dm+String(telemetry.imuData.pos_m_GRF.y)+dm+String(telemetry.imuData.pos_m_GRF.z)
  +dm+String(telemetry.imuData.vel_m_GRF.x,3)+dm+String(telemetry.imuData.vel_m_GRF.y)+dm+String(telemetry.imuData.vel_m_GRF.z)
  +dm+String(telemetry.statusWheel_A.isForward)+dm+String(telemetry.statusWheel_A.isStop)+dm+String(telemetry.statusWheel_A.pace) 
  +dm+String(telemetry.statusWheel_B.isForward)+dm+String(telemetry.statusWheel_B.isStop)+dm+String(telemetry.statusWheel_B.pace) 
  +dm+String(telemetry.statusWheel_C.isForward)+dm+String(telemetry.statusWheel_C.isStop)+dm+String(telemetry.statusWheel_C.pace) 
  +dm+String(telemetry.statusWheel_D.isForward)+dm+String(telemetry.statusWheel_D.isStop)+dm+String(telemetry.statusWheel_D.pace) 
  +dm+String(telemetry.isRoverStationary);
  ;
  return TM;
}

/*
 * Create 100 Hz ping for the telemetry loop on the second core 
 */
int createTmPing_100Hz(){
  if( loop_counter_100Hz == int( TM_LOOP_FREQUENCY_Hz / 100 ) ){
    loop_counter_100Hz = 0;
    return 1;
  } else 
  {
    loop_counter_100Hz = loop_counter_100Hz + 1;
    return 0;
  }
}

/*
 * Create 10 Hz ping for the telemetry loop on the second core 
 */
int createTmPing_10Hz(){
  if( loop_counter_10Hz == int( TM_LOOP_FREQUENCY_Hz / 10 ) ){
    loop_counter_10Hz = 0;
    return 1;
  } else 
  {
    loop_counter_10Hz = loop_counter_10Hz + 1;
    return 0;
  }
}

void assessRoverStationary(){

  if (    telemetry.statusWheel_A.isStop == true 
       && telemetry.statusWheel_B.isStop == true
       && telemetry.statusWheel_C.isStop == true
       && telemetry.statusWheel_D.isStop == true)
  {
    isRoverStationary = 1;
  }
  else
  {
    isRoverStationary = 0;
  }
  telemetry.isRoverStationary = isRoverStationary;
}

/*
 * Main function to:
 * (1) Collect sensor measurements
 * (2) Update Telemetry structure
 * (3) Publish Telemetry
 * 
 * -> This will run on the second core 
 */
void manageTelemetry( void * pvParameters ){
  /*
   * Loop TM loop forever 
   */
  int frontDistance = -1 ;
  double fVelocity  = -1 ;
  int ping_100_Hz   =  0 ;
  int ping_10Hz     =  0 ;
  int imu_meas_success = 0;
  
  for(;;)
  {
      telemetry.clockTime = telemetry.clockTime + 1;
      
      /*
       * Create Pings
       */
      ping_100_Hz = createTmPing_100Hz();
      ping_10Hz   = createTmPing_10Hz();
      
      /*
       * Collect IMU measurements
       * Runs at full TM frequency 
       */
      imu_meas_success = updateImuMeasurement();

      /*
       * Assess if the rover is stationary 
       * based on all four wheel actuator commands
       * Note: This function does not consult the IMU
       */
      assessRoverStationary();

      /*
       * 10 Hz functions 
       */
      if ( ping_10Hz == 1 )
      {
        /*
         * Collect Front laser distance measurement
         */
        if( enableDistFront )
        {
          frontDistance = measFrontDistance();
          frontDistanceTime = millis();
          fVelocity = frontVelocity( frontDistance,  prevMeasurement,  frontDistanceTime,  prevMeasurementTime);
          char dataString[8];
          dtostrf(frontDistance, 1, 2, dataString);
          telemetry.frontDistance = frontDistance ;
          if ( isBroadcastDistance )
          {
            publishData(dataString, MQTT_TOPIC_DISTANCE);
          }
        }
      
        if( enablePublishTelemetry )
        {
          /*
           * Publish telemetry to MQTT channel 
           */
          publishTelemetry();
        }
      }
    // For front sensor based velocity estimation
    prevMeasurement       = frontDistance; 
    prevMeasurementTime   = frontDistanceTime;

    delay(int( 1/TM_LOOP_FREQUENCY_Hz * 1000 ));
  }
}

/*
 * Function: Publish Telemetry to MQTT_TOPIC_TM
 */
void publishTelemetry(){

  String strTM = serializeTelemetry();
  
  if (client.publish( MQTT_TOPIC_TM, strTM.c_str() ) ) {

  } else {
    client.connect(client_id, mqtt_username, mqtt_password);
    delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
    client.publish(MQTT_TOPIC_TM, strTM.c_str() );
  }
}
