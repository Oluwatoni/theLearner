/*
 * GccApplication1.c
 *
 * Created: 2016-11-05 4:40:49 PM
 * Author : Toni
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include "usiTwiSlave.h"

#define SLAVE_ADDRESS 0x07
#define F_CPU 8000000
volatile struct{
	uint8_t pulse_count;
	uint16_t dt;
}pulses;
	
inline void activate_PB4_interrupt(){
	GIMSK |= 0b00100000;
	PCMSK |= 0b00010000;
}

inline void deactivate_PB4_interrupt(){
	GIMSK &= 0;
	PCMSK &= 0;
}

inline void activate_timer_interrupt(){
	TIMSK |= (1 << TOIE1);
}

inline void deactivate_timer_interrupt(){
	TIMSK &= 0;
}

int main(void){
	cli();
    //PB4 change interrupt setup 
	activate_PB4_interrupt();

//	set pin as an input pin
	PORTB |= (0 << PB4);
	DDRB |= (0 << DDB4);
	
	//set up timer
	TCCR1 |= (1 << 2);
	activate_timer_interrupt();
	
	pulses.pulse_count = 0;
	pulses.dt = 0;
	usiTwiSlaveInit(SLAVE_ADDRESS);
	sei();
	
    while (1){
		if(usiTwiDataInReceiveBuffer()){
			switch(usiTwiReceiveByte()){
				case 'r':
					deactivate_PB4_interrupt();
					deactivate_timer_interrupt();
					usiTwiTransmitByte(pulses.pulse_count);
					usiTwiTransmitByte(pulses.pulse_count);
					usiTwiTransmitByte(pulses.dt >> 8);
					usiTwiTransmitByte(pulses.dt & 0xFF);
					pulses.pulse_count = 0;
					pulses.dt = 0;
					activate_PB4_interrupt();
					activate_timer_interrupt();
					break;
			}
		}
	}
}

ISR(PCINT0_vect){
	cli();
	pulses.pulse_count++;
	sei();
}

ISR(TIM1_OVF_vect){
	//F_CPU/(Prescaler)*(256-Timer preset) = no of interrupts per second
	TCNT1 = 156;
	cli();
	pulses.dt++;
	sei();
}

