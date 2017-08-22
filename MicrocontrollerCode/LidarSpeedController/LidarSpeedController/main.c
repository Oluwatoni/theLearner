/*
 * LidarSpeedController.c
 *
 * Created: 2017-07-15 8:28:40 PM
 * Author : funmi
 */

#define F_CPU 20000000UL

#include "uart.h"
#include "lidar.h"

int main(void){
  lidarInit();
  sei();
  
  SetSpeed(DEFAULT_RPM);
  while (1){
    if (lidar_data_ready){
      SendLidarFrame();
    }
     //TODO after every encoder speed update correct the lidar motor output with a PID
    #ifdef PRINT_SPEED
    PrintSpeed();
    #endif
  }
}