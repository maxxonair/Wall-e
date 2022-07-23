
#ifndef ROVER_LASER_DISTANCE_H

#define ROVER_LASER_DISTANCE_H
#include "roverData.h"

// Time of flight sensor (VL53L0X)
Adafruit_VL53L0X vl53lox = Adafruit_VL53L0X();
int SHUT_VL = 12;
#define LOX1_ADDRESS 0x30
int prevMeasurement     = -1; 
int frontDistanceTime   = -1;
int prevMeasurementTime = -1;

/*
 * Function: Measure front distance with laser sensor 
 */
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


#endif
