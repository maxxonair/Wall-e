#ifndef ROVER_MQTT_H

#define ROVER_MQTT_H

#include <WiFi.h>
#include "ESP32_PubSubClient.h"
#include "Esp32MQTTClient.h"

#include "roverData.h"
#include "roverPublish.h"
#include "roverTelecommand.h"

// ================================================================
// ===                [ WIFI FUNCTIONS ]                        ===
// ================================================================
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
 * Initialize Wifi connection
 */
void wifiConnection_init(){
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
// ================================================================
// ===                    [ MQTT FUNCTIONS ]                    ===
// ================================================================

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
 *  Initialize Mqtt connection to broker
 */
void mqttConnection_init(){
  client.setServer(mqtt_server, 1883);
  //client.subscribe(MQTT_TOPIC_TC);
  client.setCallback(callback);

  connectMqtt();
 /* 
 *  Loop callback
 */
client.loop();
}

/*
 * Function: Compile and TC acknowledgement message 
 */
void publishTcAckn( String Parameter, String Value ){
  String ack_message = "[Wall-e][TM] TC valid. Setting "+Parameter+" = "+Value;
  publishData( ack_message, MQTT_TOPIC_STATUS );
}


#endif
