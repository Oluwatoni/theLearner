/*
 * GccApplication1.c
 *
 * Created: 2016-11-05 4:40:49 PM
 * Author : Toni
 */
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/cpufunc.h>
#include "usiTwiSlave.h"

#define SLAVE_ADDRESS 0x07
#define F_CPU 8000000
volatile struct{
	uint8_t pulse_count;
	uint16_t dt;
}pulses;
volatile uint8_t pin_value;
	
inline void activate_PB4_interrupt(){
	PCMSK |= 0b00010000;
	GIMSK |= 0b00100000;
}

inline void deactivate_PB4_interrupt(){
	GIMSK &= ~(0b00100000);
	PCMSK &= ~(0b00010000);
}

inline void activate_timer_interrupt(){
	TIMSK |= (1 << TOIE1);
}

inline void deactivate_timer_interrupt(){
	TIMSK &= ~(1 << TOIE1);
}

int main(void){
 //PB4 change interrupt setup 
 //	set pin as an input pin
	DDRB &= ~(1 << DDB4);
	PORTB |= (0 << PB4);
	//MCUCR |= (1 << PUD);
	activate_PB4_interrupt();
	
	//set up timer
	TCCR1 |= (1 << 2);
	
	pulses.pulse_count = 0;
	pulses.dt = 0;
	usiTwiSlaveInit(SLAVE_ADDRESS);
	sei();
  while (1){
	  if(usiTwiDataInReceiveBuffer()){
		  switch(usiTwiReceiveByte()){
			  case 'r':
				  deactivate_PB4_interrupt();
				  usiTwiTransmitByte(pulses.pulse_count);
				  pulses.pulse_count = 0;
				  activate_PB4_interrupt();
				  break;
			}
	  }
  }
}
ISR(PCINT0_vect){
	cli();
	pin_value = PINB & (1 << PB4);
	activate_timer_interrupt();
	deactivate_PB4_interrupt();
	sei();
}

ISR(TIM1_OVF_vect){
	//F_CPU/(Prescalecar = 8)*(256-Timer preset - 156) = no of interrupts per second
	TCNT1 = 156;
	cli();
	pulses.dt++;
	if(pulses.dt > 2){//2ms debounce time
    if (pin_value == (PINB & (1 << PB4)))
	    pulses.pulse_count++;
    pulses.dt = 0;
    activate_PB4_interrupt();
    deactivate_timer_interrupt();
  }
  sei();
}