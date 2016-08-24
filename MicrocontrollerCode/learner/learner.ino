/*
  Oluwatoni Ogunmade
  funmitoni2@yahoo.co.uk
  inspiration below.
  http://projectsfromtech.blogspot.com/2014/01/i2c-hc-sr04-sonar-module-attiny85-i2c.html
*/
#include <Wire.h>
#include <IMU.h>
#include <NewPing.h>
#include <Servo.h>
#include <Learner_car.h>
#include <avr/interrupt.h>
//#define DEBUG
//defines for the ultrasonic sensors
#define MCU1_I2C 1
#define MCU2_I2C 2
#define SONAR_NUM 7
#define MAX_DISTANCE 250 //in cm
#define ULTRASONIC_DELAY 14
#define SENSOR_WAVE_DELAY 20
#define PRECISE 3
#define IMU_PRINT_FREQ 15

#define UART_RX_HARDWARE_ENABLE

byte raw_distance[SONAR_NUM] = {};                   // Where the range data is stored
byte filtering_distance [SONAR_NUM][PRECISE] = {};
byte filtered_distance [SONAR_NUM] = {};
byte start_filter = 0;
int steering, throttle, stop_requested;

IMU Imu(50);//frequency in Hz
Learner_car Car;
NewPing sonar[SONAR_NUM - 4] =       // Sensor object array.
{
  NewPing(7, 8, MAX_DISTANCE), //ultra 5 // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(A2, A3, MAX_DISTANCE),//ultra 7
  NewPing(4, 2, MAX_DISTANCE)//ultra 6
};

char input_string[15];         // a string to hold incoming data
char * first_char = input_string;
uint8_t input_string_index = 0;
boolean input_string_complete = false;  // whether the string is complete
boolean gps_send = true;

void setup()
{
  Wire.begin();
  Imu.Setup();
  Car.Setup();
  Serial.begin(115200);
  sei();
  for (int i = 0; i < SONAR_NUM; i++)
  {
    filtered_distance[i] = 0;
    raw_distance[i] = 0;
    for (int j = 0; j < PRECISE; j++)
      filtering_distance[i][j] = 0;
  }
}

unsigned long temp = 0, now  = 0;

//checksum to help validate msg

uint16_t generateChecksum(char data[], byte sizeOfData)
{
  uint16_t sum = 0;
  while (sizeOfData)
  {
    sum += (uint8_t)data[--sizeOfData];
  }
  sum += 44;
  return (sum % 255);
}

void loop()
{
  if(gps_send)
    sendGPSData();
  sendUltrasonicData1();
  temp = millis();
  sendImuData();

  now = millis();
  ultrasonicDelay(now, temp);
  sendUltrasonicData2();
  temp = millis();
  sendImuData();
  
  now = millis();
  ultrasonicDelay(now, temp);
  sendUltrasonicData3();
  temp = millis();
  sendImuData();
  
  now = millis();
  ultrasonicDelay(now, temp);
  gps_send = !gps_send;
}

//handles RC msgs
void extractData(char * msg, uint8_t msg_size)
{
  int8_t state = 0, temp_steering, temp_throttle, temp_stop, checksum_size = 0,checksum;
  String raw_steering = "", raw_throttle = "", raw_stop = "", raw_checksum = "";
  char temp[15];
  
  for(int i = 0; i < msg_size; i++)
  {
    if (*msg == ',')
      state++;
    switch(state)
    {
      case 0:
        //r before the first comma
        temp[checksum_size++] = *msg;
        break;
      case 1:
        //Steering
        if (*msg != ',')
          raw_steering.concat(*msg);
        temp[checksum_size++] = *msg;
        break;
      case 2:
        //Throttle
        if (*msg != ',')
          raw_throttle.concat(*msg);
        temp[checksum_size++] = *msg;
        break;
      case 3:
        if (*msg != ',')
          raw_stop.concat(*msg);
        temp[checksum_size++] = *msg;
        break;
      case 4:
        if (*msg != '\n' && *msg != ',')
          raw_checksum.concat(*msg);
        break;
    }
    msg++;
  }
  temp[checksum_size++] = ',';
  
  if (generateChecksum(temp, checksum_size) == raw_checksum.toInt())
  {
    
    Serial.println(raw_steering);
    Serial.println(raw_throttle);
    Serial.println(raw_stop);
    
    temp_steering = raw_steering.toInt();
    temp_throttle = raw_throttle.toInt();
    temp_stop = raw_stop.toInt();
    Car.Instruct(temp_steering, temp_throttle);
  }
  /*
  else
  {
    for(int i = 0; i < checksum_size; i++)
    {
      Serial.print(temp[i]);
    }
    Serial.println();
    Serial.println(raw_checksum);
    Serial.println(generateChecksum(temp, checksum_size));
  }
  */
}

void USART_Transmit( unsigned char data )
{
  /* Wait for empty transmit buffer */
  while ( !( UCSR0A & (1<<UDRE0)) )
  ;
  /* Put data into buffer, sends the data */
  UDR0 = data;
}

//Handle incoming commands

ISR(USART_RX_vect)
{
  //USART_Transmit(UDR0);
  cli();
  char temp = UDR0;
  if (temp == '\n')
  {
    input_string[input_string_index] = temp;
    sei();
    extractData(first_char, input_string_index);
    input_string_index = 0;
    input_string_complete = true;
  }
  else
  {
    input_string_complete = false;
    input_string[input_string_index] = temp;
    input_string_index++;
  }

  sei();
}

