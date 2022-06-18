#include "RF24.h"
#include <nRF24L01.h>

void enableJoystickCtrl(RF24 radio){
        if(radio.available())  {

            while (radio.available()){      
              JoyData joyPack;
              radio.read(&joyPack, sizeof(joyPack));

              if (true) { // Debug received data 
                  Serial.print(joyPack.X1);
                  Serial.print(" -- ");
                  Serial.print(joyPack.Y1);
                  Serial.print(" -- ");
                  Serial.print(joyPack.X2);
                  Serial.print(" -- ");
                  Serial.print(joyPack.Y2);
                  Serial.println();
              }

              //-------------------------------------------------------------------
              //                  Car Speed Control
              //-------------------------------------------------------------------
              volatile boolean forward        = true;
              volatile boolean leftward       = false;
              volatile int forwardThreshold   = 5;
              
              volatile int sidewardThreshold  = 20;
              volatile int sideSpeed          = 0;
              volatile int xSpeed             = 0;
              
              // Forward command
              if( joyPack.X1 >= 8847485 ){
                xSpeed=map(joyPack.X1, 8847485, 16711800, 0, 255);
                forward = true; 
              } else {
                xSpeed=map(joyPack.X1, 8847485, 0, 0, 255);
                forward = false;
              }
              // Sideward command
              if( joyPack.Y1 >= 128 ){
                sideSpeed=map(joyPack.Y1, 128, 255, 0, 255);
                leftward = false; 
              } else {
                sideSpeed=map(joyPack.Y1, 127, 0, 0, 255);
                leftward = true ;
              }

              if ( leftward && sideSpeed > sidewardThreshold) {
                  ctrlAllWheel_RotateRightward(sideSpeed);
              } else if ( !leftward && sideSpeed > sidewardThreshold) {
                  ctrlAllWheel_RotateLeftward(sideSpeed);
              } else if ( forward && xSpeed > forwardThreshold ) {
                  ctrlAllWheel_Forward(xSpeed);
              } else if ( xSpeed > forwardThreshold ) {
                  ctrlAllWheel_Reverse(xSpeed);
              } else {
                  ctrlAllWheel_Stop();
              }
            }
          } else {
            //  Serial.print(".");
          } 
}
