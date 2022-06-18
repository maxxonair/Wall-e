
/* 
 *  MQTT Broker 
 *  
 */
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

Telemetry telemetry;
