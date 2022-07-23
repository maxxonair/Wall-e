#ifndef ROVER_PUBLISH_H

#define ROVER_PUBLISH_H

#include "roverData.h" 
/*
 * Function: Publish Data (String message) 
 */
void publishData( String data, const char* topic ){
  if (client.publish( topic, data.c_str()) ) {

  } else {
    client.connect(client_id, mqtt_username, mqtt_password);
    delay(10); // This delay ensures that client.publish doesn't clash with the client.connect call
    client.publish(topic, data.c_str());
  }
}

#endif