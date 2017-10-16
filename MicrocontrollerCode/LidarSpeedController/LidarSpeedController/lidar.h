/*
 * Lidar.h
 *
 * Created: 2017-08-21 11:33:20 PM
 *  Author: funmi
 */ 


#ifndef LIDAR_H_
#define LIDAR_H_

#define F_CPU 20000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdint-gcc.h>
#include <util/delay.h>
#include <stddef.h>

#include "string.h"
#include "uart.h"

#define DEFAULT_RPM 225

#define INCOMING_FRAME_SIZE 22
#define OUTGOING_FRAME_SIZE 19
#define MAX_QUEUE_SIZE 3

#define F_CPU 20000000UL

#define TO_MILLISEC(x) (x)*(F_CPU/1000)

volatile char incoming_frame_contents[INCOMING_FRAME_SIZE];
volatile char outgoing_frame_contents[MAX_QUEUE_SIZE][INCOMING_FRAME_SIZE];
volatile uint8_t lidar_data_ready;
volatile float rpm;

/* circular queue ADT*/
volatile int queue_index, current_queue_size;
volatile uint8_t queue_mutex;

void lidarInit();
void SendLidarFrame();
void SetSpeed(uint8_t);
void PrintSpeed();

#endif /* LIDAR_H_ */