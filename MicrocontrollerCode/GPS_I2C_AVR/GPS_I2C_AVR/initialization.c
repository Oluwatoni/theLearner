/*
 * initialization.c
 *
 * Created: 08/04/2016 6:07:44 PM
 *  Author: Toni
 */ 
#include "initialization.h"

void USART_Init( unsigned int ubrr)
{
	/*Set baud rate */
	UBRR0H = (unsigned char)(ubrr>>8);
	UBRR0L = (unsigned char)ubrr;
	
	/*Enable receiver and transmitter and RX interrupt */
	UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
	/* Set frame format: 8data, 1stop bit */
	UCSR0C = (3<<UCSZ00);
}

void USART_Transmit( unsigned char data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
	;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}

unsigned char USART_Receive( void )
{
	/* Wait for data to be received */
	while ( !(UCSR0A & (1<<RXC0)) )
	;
	/* Get and return received data from buffer */
	return UDR0;
}

unsigned char TWI_Act_On_Failure_In_Last_Transmission ( unsigned char TWIerrorMsg )
{
	// A failure has occurred, use TWIerrorMsg to determine the nature of the failure
	// and take appropriate actions.
	// Se header file for a list of possible failures messages.
	
	// This very simple example puts the error code on PORTB and restarts the transceiver with
	// all the same data in the transmission buffers.
	PORTB = TWIerrorMsg;
	TWI_Start_Transceiver();
	
	return TWIerrorMsg;
}

uint8_t compareCharArray(char a[], char b[], uint8_t size)
{
	for (uint8_t i = 0;i < size; i++)
	{
		if (a[i] != b[i])
			return 0;
	}
	return 1;
}

/*
//Returns the amount of units to increment the pointer
uint8_t fillCharArray(char array[], char* pointer, uint8_t size, uint8_t no_of_commas)
{
	uint8_t move_pointer_by = 0;
	for(int i = 0; i <= size; i++)
	{
		if(*pointer == ',' || *pointer == '\n')
		{
			no_of_commas--;
			if (!no_of_commas)
			{
				move_pointer_by++;
				break;
			}
		}
		array[i] = (char)*pointer;
		pointer++;
		move_pointer_by++;
	}
	return move_pointer_by;
}
*/

//Returns the amount of units to increment the pointer and takes a pointer to a checksum
uint8_t fillCharArray(char array[], char* pointer, uint8_t size, uint8_t no_of_commas, gps_data_type_t type)
{
  uint8_t move_pointer_by = 0;
  uint8_t total = 0;

  for(total; total <= size; total++)
  {
    if(*pointer == ',' || *pointer == '\n')
    {
      no_of_commas--;
      if (!no_of_commas)
      {
        move_pointer_by++;
        break;
      }
    }
    array[total] = (char)*pointer;
    pointer++;
    move_pointer_by++;
  }

  if(type != NONE)
    I2C_checksum_array[type] = (char)(total+'0');
  
  return move_pointer_by;
}

void printCharArray(char array[], uint8_t size)
{
	for (uint8_t i = 0;i < size; i++)
	{
		USART_Transmit(*(array++));
	}
	USART_Transmit('\n');
}

//TODO CHECKSUM CHECK
void extractData(char* message_index)
{
	char message_type[MESSEAGE_TYPE_SIZE] = {'E','R','R'};
	char gprmc[] = {'G','P','R','M','C'};
	char gpgga[] = {'G','P','G','G','A'};
	char temp_time[TIME_SIZE] = {'E','R','6'};
	char temp_latitude[LATITUDE_SIZE] = {'E','R','1'};
	char temp_longitude[LONGITUDE_SIZE] = {'E','R','2'};
	char temp_speed[SPEED_SIZE] = {'E','R','3'}; //in knots
	char temp_course[COURSE_SIZE] = {'E','R','4'};
	char temp_altitude[ALTITUDE_SIZE] = {'E','R','5'};
	char temp[TEMP_SIZE];
	char message_checksum[GPS_CHECKSUM_SIZE];
	
	if (*message_index == '$')
	{		
		message_index++;
		message_index += fillCharArray(message_type, message_index, MESSEAGE_TYPE_SIZE,1, NONE);
		if (compareCharArray(message_type, gprmc, MESSEAGE_TYPE_SIZE))
		{
			//sample gprmc "$GPRMC,054313.80,A,4328.72490,N,08032.14223,W,0.075,,090416,,,A*68"

			message_index += fillCharArray(temp_time, message_index, TIME_SIZE,1, TIME);

			if(*message_index == 'A')
			{
				message_index += 2;
				message_index += fillCharArray(temp_latitude, message_index, LATITUDE_SIZE,2, LAT);
				message_index += fillCharArray(temp_longitude, message_index, LONGITUDE_SIZE,2, LONG);
				message_index += fillCharArray(temp_speed, message_index, SPEED_SIZE,1, SPD);
				message_index += fillCharArray(temp_course, message_index, COURSE_SIZE,1, CRSE);
				message_index += fillCharArray(temp, message_index, TEMP_SIZE, 3, NONE);
				message_index += 2;
				fillCharArray(message_checksum, message_index, GPS_CHECKSUM_SIZE, 1, NONE);				
			}
			/*
			printCharArray(message_type, MESSEAGE_TYPE_SIZE);
			printCharArray(temp_time, TIME_SIZE);
			printCharArray(temp_latitude, LATITUDE_SIZE);
			printCharArray(temp_longitude, LONGITUDE_SIZE);
			printCharArray(temp_speed, SPEED_SIZE);
			printCharArray(temp_course, COURSE_SIZE);
			printCharArray(message_checksum,GPS_CHECKSUM_SIZE);
			*/
      
      memcpy(time, temp_time, TIME_SIZE);
      memcpy(latitude, temp_latitude, LATITUDE_SIZE);
      memcpy(longitude, temp_longitude, LONGITUDE_SIZE);
      memcpy(altitude, temp_altitude, ALTITUDE_SIZE);
      memcpy(speed, temp_speed, SPEED_SIZE);
      memcpy(course, temp_course, COURSE_SIZE);
		}
		else if(compareCharArray(message_type,gpgga,MESSEAGE_TYPE_SIZE))
		{
			//sample gpgga "$GPGGA,054313.80,4328.72490,N,08032.14223,W,1,09,0.97,344.8,M,-36.0,M,,*68"
			message_index += fillCharArray(temp, message_index, TEMP_SIZE, 8, NONE);
			fillCharArray(temp_altitude, message_index, ALTITUDE_SIZE,1, ALT);
			
      /*
			printCharArray(message_type, MESSEAGE_TYPE_SIZE);
			printCharArray(temp_altitude, ALTITUDE_SIZE);
			*/
      
      memcpy(altitude, temp_altitude, ALTITUDE_SIZE);
		}
	}
}

void prepare_TWI_data (gps_data_type_t type, char* data, uint8_t size)
{
  //printCharArray(data,size);
  //TWI_Start_Transceiver_With_Data(data, size);
  
  unsigned char include_checksum[size+1];
  memcpy(include_checksum,data, size);
  include_checksum[size] = I2C_checksum_array[type];
  TWI_Start_Transceiver_With_Data(include_checksum, size+1);
  //printCharArray(include_checksum, size+1);
  
}

ISR(USART_RX_vect)
{
	cli();
	char temp = UDR0;
	if (temp == '\n')
	{
		received_data[usart_index] = temp;
		usart_index = 0;
		sei();
		extractData(first_char);
		data_is_ready = 1;
	}
	else
	{
		data_is_ready = 0;
		received_data[usart_index] = temp;
		usart_index++;
	}
	sei();
}
