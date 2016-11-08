/*
  Oluwatoni Ogunmade
  funmitoni2@yahoo.co.uk
  inspiration below.
  http://projectsfromtech.blogspot.com/2014/01/i2c-hc-sr04-sonar-module-attiny85-i2c.html
*/
#include <Wire.h>
#include <NewPing.h>
#include <Servo.h>
#include <avr/interrupt.h>
#include <Cristians_clock.h>
#include <IMU.h>
#include <Learner_car.h>

#define UART_RX_HARDWARE_ENABLE

//#define DEBUG
//defines for the ultrasonic sensors
#define MCU1_I2C 1
#define MCU2_I2C 2
#define SONAR_NUM 7
#define MAX_DISTANCE 250 //in cm
#define ULTRASONIC_DELAY 13
#define SENSOR_WAVE_DELAY 20
#define PRECISE 3

byte raw_distance[SONAR_NUM] = {};                   // Where the range data is stored
byte filtering_distance [SONAR_NUM][PRECISE] = {};
byte filtered_distance [SONAR_NUM] = {};
byte start_filter = 0;
int steering, throttle, stop_requested, time_counter = 5;

IMU Imu(50);//frequency in Hz
Learner_car Car;
Clock sensor_clock;

NewPing sonar[SONAR_NUM - 4] =       // Sensor object array.
{
  NewPing(7, 7, MAX_DISTANCE), //ultra 5 // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(A2, A2, MAX_DISTANCE),//ultra 7
  NewPing(2, 2, MAX_DISTANCE)//ultra 6
};

char input_string[40];         // a string to hold incoming data
uint8_t input_string_index = 0;
boolean input_string_complete = false;  // whether the string is complete

void setup()
{
  Wire.begin();
  Imu.Setup();
  Car.Setup();
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
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

void loop()
{
  if (time_counter == 5)
  {
    time_counter = 0;
    sensor_clock.requestTime();
  }
  sendAccData();
  sendImuData();
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
  time_counter++;
}

//handles RC msgs
void extractData(char * msg, uint8_t msg_size)
{
  int8_t state = 0, temp_steering, temp_throttle, temp_stop, checksum_size = 0, checksum;
  String raw_steering = "", raw_throttle = "", raw_stop = "", raw_checksum = "";
  char temp[15];

  for (int i = 0; i < msg_size; i++)
  {
    if (*msg == ',')
      state++;
    switch (state)
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
#ifdef DEBUG
    Serial.println(raw_steering);
    Serial.println(raw_throttle);
    Serial.println(raw_stop);
#endif

    temp_steering = raw_steering.toInt();
    temp_throttle = raw_throttle.toInt();
    temp_stop = raw_stop.toInt();
    Car.Instruct(temp_steering, temp_throttle);
  }
#ifdef DEBUG
  else
  {
    for (int i = 0; i < checksum_size; i++)
    {
      Serial.print(temp[i]);
    }
    Serial.println();
    Serial.println(raw_checksum);
    Serial.println(generateChecksum(temp, checksum_size));
  }
#endif
}

//using Cristian's algorithm to update the arduino's time
void recieveTime(char * msg, uint8_t msg_size)
{
  uint8_t state = 0, checksum_size = 0, checksum;
  uint32_t int_second, int_microsecond;
  String raw_second = "", raw_microsecond = "", raw_checksum = "";
  char temp[40];

  for (int i = 0; i < msg_size; i++)
  {
    if (*msg == ',')
      state++;
    switch (state)
    {
      case 0:
        //t before the first comma
        temp[checksum_size++] = *msg;
        break;
      case 1:
        if (*msg != ',')
          raw_second.concat(*msg);
        temp[checksum_size++] = *msg;
        break;
      case 2:
        if (*msg != ',')
          raw_microsecond.concat(*msg);
        temp[checksum_size++] = *msg;
        break;
      case 3:
        if (*msg != '\n' && *msg != ',')
          raw_checksum.concat(*msg);
        break;
    }
    msg++;
  }
  temp[checksum_size++] = ',';

  if (generateChecksum(temp, checksum_size) == raw_checksum.toInt())
  {
    long recieved_time = micros();
#ifdef DEBUG
    Serial.println("Valid time message");
    Serial.println(raw_second);
    Serial.println(raw_microsecond);
#endif
  int_second = raw_second.toInt();
  int_microsecond = raw_microsecond.toInt();
  
    int_microsecond = int_microsecond;
    if (int_microsecond > 1000000)
    {
      int_microsecond %= 1000000;
      int_second+= int_microsecond /1000000;
    }
    sensor_clock.setTime(int_second, int_microsecond);
  }
#ifdef DEBUG
  else
  {
    Serial.println("Invalid time message");
    for (int i = 0; i < checksum_size; i++)
      Serial.print(temp[i]);

    Serial.println();
    Serial.println(raw_checksum);
    Serial.println(generateChecksum(temp, checksum_size));
    Serial.println("Bye");
  }
#endif
}


//Handle incoming commands
ISR(USART_RX_vect)
{
  cli();
  char temp = UDR0;
  if (temp == '\n')
  {
    input_string[input_string_index] = temp;
    sei();
    if (input_string[0] == 'r')
      extractData(&input_string[0], input_string_index);
    else if (input_string[0] == 't')
      recieveTime(&input_string[0], input_string_index);
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
