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

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup()
{
  Imu.Setup();
  Car.Setup();
  Serial.begin(115200);
  sei();
  UCSR0B = (1<<RXEN0)|(1<<TXEN0)|(1<<RXCIE0);
  
  for (int i = 0; i < SONAR_NUM; i++)
  {
    filtered_distance[i] = 0;
    raw_distance[i] = 0;
    for (int j = 0; j < PRECISE; j++)
      filtering_distance[i][j] = 0;
  }
}

unsigned long temp = 0, now  =0;
float freq;
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
  sendUltrasonicData1();
  temp = millis();
  sendImuData();
  Car.Instruct(steering,throttle);
  now = millis();
  ultrasonicDelay(now,temp);
  sendUltrasonicData2();
  temp = millis();
  sendImuData();
  Car.Instruct(steering,throttle);
  now = millis();
  ultrasonicDelay(now,temp);
  sendUltrasonicData3();
  temp = millis();
  sendImuData();
  Car.Instruct(steering,throttle);
  now = millis();
  ultrasonicDelay(now,temp);
  
}

//handles RC msgs
void serialEvent() {
  /*
  int temp_steering, temp_throttle, temp_stop_requested,checksum;
  String to_be_checked = "";
  inputString = "";
    delay(10);
    inputString = Serial.readStringUntil(',');
    if (inputString.startsWith("r"))
    {
      to_be_checked.concat("r,");
      temp_steering = (inputString.substring(1)).toInt();
      to_be_checked.concat(temp_steering);
      to_be_checked.concat(",");
      inputString = Serial.readStringUntil(',');
      temp_throttle = inputString.toInt();
      inputString = Serial.readStringUntil(',');
      to_be_checked.concat(temp_throttle);
      to_be_checked.concat(",");
      temp_stop_requested = inputString.toInt();
      to_be_checked.concat(temp_stop_requested);
      to_be_checked.concat(",");
      inputString = Serial.readStringUntil('\n');
//      Serial.println(to_be_checked);
      checksum = inputString.toInt();

    }
    char buffer[to_be_checked.length()];
    to_be_checked.toCharArray(buffer, to_be_checked.length());
    if (checksum == generateChecksum(buffer, to_be_checked.length()))
    {
#ifdef DEBUG
      Serial.println("\n\n\n");
      String debug_msg = "";
      debug_msg.concat(steering);
      debug_msg.concat(",");
      debug_msg.concat(throttle);
      Serial.println(debug_msg);
      Serial.println("\n\n\n");
#endif
      steering = temp_steering;
      throttle = temp_throttle;
      stop_requested = temp_stop_requested;
    }
    /*
    else
    {
      Serial.println("\n\n\n");
      Serial.println(to_be_checked);
      Serial.print(generateChecksum(buffer, to_be_checked.length()));
      Serial.print(", ");
      Serial.println(checksum);
      Serial.println("\n\n\n");
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

/*
ISR(USART_RX_vect)
{
  USART_Transmit(UDR0);
  cli();
  /*
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
*/

