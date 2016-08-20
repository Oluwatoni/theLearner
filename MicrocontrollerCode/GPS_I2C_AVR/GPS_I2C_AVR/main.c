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
  unsigned char request = 'i';
	
  
	while (1) 
  {
		if ( !TWI_Transceiver_Busy() )
		{
			if ( TWI_statusReg.RxDataInBuf )
			{
				TWI_Get_Data_From_Transceiver(&request, 1); 
        TWI_Start_Transceiver();
				switch (request)
				{
					//a for latitude
					case 'a':
            prepare_TWI_data(LAT,latitude,LATITUDE_SIZE);
						break;
					
          //b for longitude
					case 'b':
            prepare_TWI_data(LONG,longitude,LONGITUDE_SIZE);
						break;
					
          //c for time
					case 'c':
            prepare_TWI_data(TIME,time,TIME_SIZE);
						break;
					
          //d for speed
					case 'd':
            prepare_TWI_data(SPD,speed,SPEED_SIZE);
						break;
					
          //e for course
					case 'e':
            prepare_TWI_data(CRSE,course,COURSE_SIZE);
						break;
					
          //f for altitude
					case 'f':
            prepare_TWI_data(ALT,altitude,ALTITUDE_SIZE);
						break;
            
					default:
            TWI_Start_Transceiver_With_Data(NULL, NULL);
						break;
				} 
			}
		}
    request = 'i';
  }
}
