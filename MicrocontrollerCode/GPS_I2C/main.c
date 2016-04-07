#define F_CPU 1000000  // CPU frequency for proper time calculation in delay function

#include <avr/io.h>
#include <util/delay.h>

int main(void)
{
    DDRD |= (1 << PD6);  // make PD6 an output

    for(;;)
    {
        PORTD ^= (1 << PD6);  // toggle PD6
        _delay_ms(1000);  // delay for a second
    }

    return 0;  // the program executed successfully
}
