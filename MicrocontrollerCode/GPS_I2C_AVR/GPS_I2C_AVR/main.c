/*
 * GPS_I2C_AVR.c
 *
 * Created: 01/04/2016 2:29:38 AM
 * Author : Toni
 */ 
#include "initialization.h"

int main(void)
{
	// initialize as slave
	USART_Init(MYUBRR);
	unsigned char TWI_slaveAddress;
		
	// Own TWI slave address
	TWI_slaveAddress = 0x10;
	
	// Initialize TWI module for slave operation. Include address and/or enable General Call.
	TWI_Slave_Initialise( (unsigned char)((TWI_slaveAddress<<TWI_ADR_BITS) | (TRUE<<TWI_GEN_BIT) ));
	// Start the TWI transceiver to enable reception of the first command from the TWI Master.
	sei();
	TWI_Start_Transceiver();
  char temp = 'i';
	
	while (1) 
  {
		if ( !TWI_Transceiver_Busy() )
		{
			if ( TWI_statusReg.RxDataInBuf )
			{
				TWI_Get_Data_From_Transceiver(&temp, 1); 
				switch (temp)
				{
					//a for latitude
					case 'a':
						TWI_Start_Transceiver_With_Data(latitude, LATITUDE_SIZE);
						printCharArray(latitude,LATITUDE_SIZE);
						break;
					//b for longitude
					case 'b':
						TWI_Start_Transceiver_With_Data(longitude, LONGITUDE_SIZE);
						printCharArray(longitude,LONGITUDE_SIZE);
						break;
					//c for time
					case 'c':
						TWI_Start_Transceiver_With_Data(time, TIME_SIZE);
						printCharArray(time, TIME_SIZE);
						break;
					//d for speed
					case 'd':
						TWI_Start_Transceiver_With_Data(speed, SPEED_SIZE);
						printCharArray(speed, SPEED_SIZE);
						break;
					//e for course
					case 'e':
						TWI_Start_Transceiver_With_Data(course, COURSE_SIZE);
						printCharArray(course, COURSE_SIZE);
						break;
					//f for altitude
					case 'f':
						TWI_Start_Transceiver_With_Data(altitude, ALTITUDE_SIZE);
						printCharArray(altitude, ALTITUDE_SIZE);
						break;
					default:
            TWI_Start_Transceiver_With_Data(altitude, ALTITUDE_SIZE);
						break;
				} 
			}
		}
  }
}
