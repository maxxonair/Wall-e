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
#include <MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include "StringSplitter.h"

// MQTT
#include "Esp32MQTTClient.h"
// Include Wifi access codes
#include "wifiAccessCodes.h"
#include "roverData.h" 
#include "roverTelemetry.h"
#include "IMU.h"
#include "roverMotorCntrl.h"
#include "roverJoystickCntrl.h"
#include "roverSensorFunction.h"

//------------------------------------------------------------------------------
// Define Drive Modes:
//------------------------------------------------------------------------------ 
 const int IDLE_MODE            = 0;
 const int DEMO_SINGLE_WHEEL    = 1;
 const int DEMO_ALL_WHEEL       = 2;
 const int GAMEPAD_REMOTE_CTRL  = 3;
 const int DEMO_AUTO_DRIVE_1    = 4;
//------------------------------------------------------------------------------
// !!! Set Drive Mode:
//------------------------------------------------------------------------------
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
 int TM_LOOP_FREQUENCY_Hz    = 100;

 int TM_PUBLISH_FREQUENCY_HZ =  10;

 /*
  * Core ID at which Telemetry task will be executed
  */
 static int TM_TASK_AT_CORE_ID   =   1;
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
//------------------------------------------------------------------------------
// Time of flight sensor (VL53L0X)
Adafruit_VL53L0X vl53lox = Adafruit_VL53L0X();
int SHUT_VL = 12;
#define LOX1_ADDRESS 0x30
int prevMeasurement     = -1; 
int frontDistanceTime   = -1;
int prevMeasurementTime = -1;
//------------------------------------------------------------------------------
// Radio Communication
int CE  = 4;
int CSN = 5;
RF24 radio(CE,CSN);
const uint64_t NRF24_PIPE_ADR = 0xE8E8F0F0E1LL;
//------------------------------------------------------------------------------
// Wifi
// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;
//PubSubClient client(wifiClient);
PubSubClient client(mqtt_server, port, wifiClient); 

// Set static IP address
IPAddress local_IP(192, 168, 1, 189); 
// Set Gateway IP address
IPAddress gateway(192, 168, 1, 1);

IPAddress subnet(255, 255, 0, 0);
IPAddress primaryDNS(79, 79, 79, 77);   //optional
IPAddress secondaryDNS(79, 79, 79, 78); //optional

const String thisHostname = "walle";
//------------------------------------------------------------------------------
//------------------------------------------------------------------------------
// Constants
//------------------------------------------------------------------------------
String TC_IDENTIFIER = "[TC]";
String TC_SET_VEHICLE_MODE              = "SET_VEHICLE_MODE";
String TC_BROADCAST_DIST                = "SET_BROADCAST_DIST";
String TC_SET_AUTO_PACE                 = "SET_AUTO_PACE";
String TC_ENABLE_TM                     = "SET_ENABLE_TM";

TaskHandle_t TelemetryTask;
//------------------------------------------------------------------------------
// Internal Status variables
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
int autoDrivingPace = 80;

/* 
 *  Time since radio is without contact
 */
long timeOffRadio = 0 ; 

long connection_failures        = 0;
long broker_connection_failures = 0;

void manageTelemetry( void * pvParameters );

void setup() {
  // Setup TM serial
  Serial.begin(tm_baud);
  delay(10);
  
  Serial.println();
  Serial.println("RoverOne mk1 init ... ");
  //publishData("RoverOne mk1 init ... ", MQTT_TOPIC_STATUS);
    
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
   * INIT WIFI
   */
  if ( enableWifi ) {
      WiFi.mode(WIFI_STA);
      WiFi.hostname(thisHostname.c_str());
    
      // Configures static IP address
      if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
        Serial.println("[WIFI] STA Failed to configure");
      }
      /* 
       *  If not connected to wifi -> connect
       */
      if ( WiFi.status() != WL_CONNECTED)
      {
          if ( setup_wifi() == 0 ) {
            connection_failures = connection_failures + 1;
          }
      } 
  }

  /* 
   *  Init MQTT
   */
  if ( enableMqtt) 
  {
      client.setServer(mqtt_server, 1883);
      //client.subscribe(MQTT_TOPIC_TC);
      client.setCallback(callback);

      connectMqtt();
     /* 
     *  Loop callback
     */
    client.loop();
  }

  // INIT MPU6050
  if ( enableIMU && !isSkipSensorInit ){
    Serial.println("INIT: IMU Sensor MPU6050 ...");
    //publishData("INIT: IMU Sensor MPU6050 ...",MQTT_TOPIC_STATUS);
    // initialize device
    mpu.initialize();
    //mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    //mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    //mpu.setFilterBandwidth(MPU6050_BAND_184_HZ);
    /*
    if ( !mpu.begin() ) {
      Serial.println("Failed to find MPU6050");
    }
    */
  Serial.println(">> MPU6050 Sensor Online.");
  }
  
  // INIT VL53L0X 
  if (enableDistFront && !isSkipSensorInit){
      // all reset
    digitalWrite(SHUT_VL, LOW);    
    delay(10);
    // all unreset
    digitalWrite(SHUT_VL, HIGH);
    delay(10);
    Serial.println("INIT: Front Distance Sensor VL53L0X ...");
    if (!vl53lox.begin(LOX1_ADDRESS)) {
      Serial.println(F("Failed to boot VL53L0X"));
    }
    Serial.println(">> VL53L0X Sensor Online."); 
  } else if (enableDistFront) {
    vl53lox.begin(LOX1_ADDRESS);
  }
 
  // INIT NRF24
  if (enableRadio) {
    Serial.println("INIT: NRF24");
    radio.begin();
    
    //set the address
    radio.openReadingPipe(0, NRF24_PIPE_ADR);
    
    //Set module as receiver
    radio.startListening();
  }

  /*
   * From here onwards publishData can be used
   */
  publishData(String("[Wall-e][TM] Switching ON."), MQTT_TOPIC_STATUS);
  publishData(String("[Wall-e][TM] Execute come to life routine."), MQTT_TOPIC_STATUS);

  /*
   * Setup Telemetry on the seconds core 
   */
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
  Serial.println(" >> Execute come to life routine.");
  /*
   * Quick shake to signal:
   * (a) Automotive functions are operational  
   * (b) Intialization has been completed successfully
   * 
   */
  comeToLifeRoutine();
  
  // Set all wheels >> STOP
  Serial.println(" >> Set all motor control to STOP");
  digitalWrite(inA1, HIGH);
  digitalWrite(inA2, HIGH);
  digitalWrite(inB1, HIGH);
  digitalWrite(inB2, HIGH);
  digitalWrite(inC1, HIGH);
  digitalWrite(inC2, HIGH);
  digitalWrite(inD1, HIGH);
  digitalWrite(inD2, HIGH);

  long timeOffRadio = 0 ; 

  publishData(String("[Wall-e][TM] ... Initialization complete!"), MQTT_TOPIC_STATUS);
  Serial.println("... Initialization complete!");
  Serial.println();
}

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
                performPointTurn();              
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

void comeToLifeRoutine(){
  // Shake 30 Hz for 2 seconds
  shake(40,1.5);
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

void setDrivingMode(int newMode){
  VehicleMode       = newMode;
  telemetry.drivingMode = newMode;
  publishData( String("[Wall-e][TM] Change Vehicle Mode to: ")+String(newMode), MQTT_TOPIC_STATUS);
}

int measFrontDistance(){
  int distance = -1;
  if(enableDistFront)
  {
      VL53L0X_RangingMeasurementData_t measure;
      vl53lox.rangingTest(&measure, false);   
       
      if (measure.RangeStatus != 4) 
      {  
        distance = measure.RangeMilliMeter;
      } 
  }
  telemetry.frontDistance  = distance;
  return distance;
}

double frontVelocity(int fDis, int fDisPrev, int fDisTime, int fDisPrevTime){
  double vel = -1;
  if(fDis != -1 && fDisPrev != -1 && fDisTime != -1 && fDisPrevTime != -1 && (fDisTime - fDisPrevTime) > 0 ){
    vel = ( (double) (fDisPrev - fDis) ) / ((double) (fDisTime - fDisPrevTime) ) ;
  }
  return vel;
}

/*
 * Function: Setup Wifi connection
 */
int  setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.print("[WIFI] Connecting to ");
  Serial.println(ssid);
  Serial.print("[WIFI] Host name: ");
  Serial.println(thisHostname);
  Serial.print("[WIFI] D1 Mac Address: ");
  Serial.println(WiFi.macAddress());

  WiFi.begin(ssid, wifi_password);
  int connectionCounter = 0 ;
  int delayIncrement_ms = 250;
  
  while (WiFi.status() != WL_CONNECTED) {

  }

  Serial.print("[WIFI] Connected. IP address: ");
  Serial.println(WiFi.localIP());
  return 1;
}

  /* 
 *  Function: Re-connect to MQTT server
 */
int connectMqtt(){
  Serial.println("[MQTT] Connecting to Mqtt broker ... ");
  if (WiFi.status() == WL_CONNECTED) {
      while (!client.connected()) {
      
          if (client.connect(client_id, mqtt_username, mqtt_password)) 
          {
            Serial.println("[MQTT] Connected to MQTT Broker!");
            // Subscribe
            client.subscribe(MQTT_TOPIC_TC);
            
            return 1 ;
          }
          else 
          {
            Serial.println("[MQTT] Connection to MQTT Broker failed!");
            return 0 ;
          }

          delay(1000);
      }
  } else {
    Serial.println("[MQTT] No Wifi connection. Aborting");
    return 0;
  }
}

/*
 * Function: Compile and TC acknowledgement message 
 */
void publishTcAckn( String Parameter, String Value ){
  String ack_message = "[Wall-e][TM] TC valid. Setting "+Parameter+" = "+Value;
  publishData( ack_message, MQTT_TOPIC_STATUS );
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
        publishTcAckn(parameter_iden, value_iden);
        processTelecommand(parameter_iden, value_iden);
      }
  } 
}

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
  else
  {
    publishData( String("[Wall-e][TM] TC not valid. Rejecting. "), MQTT_TOPIC_STATUS );
  }

}

//-------------------------------------------------------------------------
/*
 *  MQTT message callback
 */
void callback(char* topic, byte* message, unsigned int length) {
  
  String messageTemp;
  for (int i = 0; i < length; i++) {
    messageTemp += (char)message[i];
  }
  
  publishData( String("[Wall-e][TM] Message received. Topic: ")+String(topic)+String(" Message: ")+String(messageTemp), MQTT_TOPIC_STATUS );

  /* Topic: Telecommand */
  parseAndProcessTelecommand(String(messageTemp));
  
}

/*
 * Function Serialize Telemetry structure to csv row
 */
String serializeTelemetry(){
  String TM = "";
  String dm = ";";
  TM = "[TM]"+dm+String(telemetry.clockTime)+dm+String(telemetry.drivingMode)+dm+String(telemetry.pointTurnStatus)
  +dm+String(telemetry.frontDistance)+dm+String(telemetry.imuData.temperature)
  +dm+String(telemetry.imuData.ax)+dm+String(telemetry.imuData.ay)+dm+String(telemetry.imuData.az)
  +dm+String(telemetry.imuData.gx)+dm+String(telemetry.imuData.gy)+dm+String(telemetry.imuData.gz)
  +dm+String(telemetry.statusWheel_A.isForward)+dm+String(telemetry.statusWheel_A.isStop)+dm+String(telemetry.statusWheel_A.pace) 
  +dm+String(telemetry.statusWheel_B.isForward)+dm+String(telemetry.statusWheel_B.isStop)+dm+String(telemetry.statusWheel_B.pace) 
  +dm+String(telemetry.statusWheel_C.isForward)+dm+String(telemetry.statusWheel_C.isStop)+dm+String(telemetry.statusWheel_C.pace) 
  +dm+String(telemetry.statusWheel_D.isForward)+dm+String(telemetry.statusWheel_D.isStop)+dm+String(telemetry.statusWheel_D.pace) ;
  return TM;
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
  int step_until_publish = int (TM_LOOP_FREQUENCY_Hz / TM_PUBLISH_FREQUENCY_HZ);
  int step_counter       = 0;
  for(;;)
  {
      telemetry.clockTime = telemetry.clockTime + 1;
      
      /*
       * Collect IMU measurements
       */
      ImuData imuData = measImu();
      
      int frontDistance = -1;
      double fVelocity  = -1;
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
    
    if( enablePublishTelemetry && step_counter >= step_until_publish )
    {
      /*
       * Publish telemetry to MQTT channel 
       */
      publishTelemetry();
      step_counter = 0;
    }

    // For front sensor based velocity estimation
    prevMeasurement     = frontDistance; 
    prevMeasurementTime = frontDistanceTime;
    step_counter = step_counter + 1;
    delay(int( 1/TM_LOOP_FREQUENCY_Hz * 1000 ));
  }
}

/*
 * Function: Publish Data (String message) 
 */
void publishData( String data, const char* topic ){
  if (client.publish( topic, data.c_str()) ) {
    Serial.print("Data Package: ");
    Serial.print(data);
    Serial.println(" sent!");
  } else {
    Serial.println("Data package failed to send. Reconnecting to MQTT Broker and trying again");
    client.connect(client_id, mqtt_username, mqtt_password);
    delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
    client.publish(topic, data.c_str());
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

/*
 * Execute point turn with random angle and direction
 */
void performPointTurn( ){
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
