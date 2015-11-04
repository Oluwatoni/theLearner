//Oluwatoni Ogunmade
///////////////////////////////////////////////////////////////////////////////////
//*********************************LIBRARIES USED*********************************//
///////////////////////////////////////////////////////////////////////////////////
#include <Servo.h>
#include <NewPing.h>
#include <Wire.h>
#include <MemoryFree.h>
//#include <avr/wdt.h>
//#include <avr/io.h>
//#include <avr/interrupt.h>
///////////////////////////////////////////////////////////////////////////////////
//***************************GLOBAL VARIABLES & DEFINES***************************//
///////////////////////////////////////////////////////////////////////////////////
int direction;
int batteryLevel;

//TODO remove
boolean output_errors = false;  // true or false

//Drive pin numbers
#define FORWARD_PIN 5
#define REVERSE_PIN 6
#define STEERING_SERVO_PIN 3
#define TEST_LED_PIN 13
#define BATTERY_MONITOR_PIN A7
//Ultrasonic setup
#define SONAR_NUM     4 // Number or sensors.
#define MAX_DISTANCE 300 // Maximum distance (in cm) to ping.
#define PING_INTERVAL 30 // Milliseconds between sensor pings (29ms is about the min to avoid cross-sensor echo).

unsigned long pingTimer[SONAR_NUM]; // Holds the times when the next ping should happen for each sensor.
unsigned int cm[SONAR_NUM];         // Where the ping distances are stored.
uint8_t currentSensor = 0;          // Keeps track of which sensor is active.
int front, left, right, rear, j = 0; //ultrasonic sensor values
const int PRECISE = 40;

//arrays to sort the raw values from the ultrasonc sensors
int frontReal[PRECISE] = {};//arrays to sort the raw values from the ultrasonc
int leftReal[PRECISE] = {};
int rightReal[PRECISE] = {};
int rearReal[PRECISE] = {};

NewPing sonar[SONAR_NUM] =       // Sensor object array.
{
  NewPing(7, 8, MAX_DISTANCE), // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(A2, A3, MAX_DISTANCE),
  NewPing(12, 11, MAX_DISTANCE),
  NewPing(4, 2, MAX_DISTANCE),
};
//IMU
#define OUTPUT__DATA_INTERVAL 35
//TODO remove the define
#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)
long lastPoll = 0;
// Accelerometer
// "accel x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define ACCEL_X_MIN ((float) -283)
#define ACCEL_X_MAX ((float) 247)
#define ACCEL_Y_MIN ((float) -258)
#define ACCEL_Y_MAX ((float) 270)
#define ACCEL_Z_MIN ((float) -297)
#define ACCEL_Z_MAX ((float) 238)

// Magnetometer (standard calibration mode)
// "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
#define MAGN_X_MIN ((float) -600)
#define MAGN_X_MAX ((float) 600)
#define MAGN_Y_MIN ((float) -600)
#define MAGN_Y_MAX ((float) 600)
#define MAGN_Z_MIN ((float) -600)
#define MAGN_Z_MAX ((float) 600)

// Magnetometer (extended calibration mode)
// Uncommend to use extended magnetometer calibration (compensates hard & soft iron errors)
#define CALIBRATION__MAGN_USE_EXTENDED true
const float magn_ellipsoid_center[3] = {61.1818, 17.4708, -281.550};
const float magn_ellipsoid_transform[3][3] = {{0.913998, 0.00713610, -0.0348895},
  {0.00713610, 0.873824, 0.0566169},
  { -0.0348895, 0.0566169, 0.962865}
};
// Gyroscope
// "gyro x,y,z (current/average) = 41.00/-41.93  -26.00/-26.52  5.00/4.26
#define GYRO_AVERAGE_OFFSET_X ((float) -42.88)
#define GYRO_AVERAGE_OFFSET_Y ((float) -29.52)
#define GYRO_AVERAGE_OFFSET_Z ((float) 7.08)

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET ((ACCEL_X_MIN + ACCEL_X_MAX) / 2.0f)
#define ACCEL_Y_OFFSET ((ACCEL_Y_MIN + ACCEL_Y_MAX) / 2.0f)
#define ACCEL_Z_OFFSET ((ACCEL_Z_MIN + ACCEL_Z_MAX) / 2.0f)
#define ACCEL_X_SCALE (GRAVITY / (ACCEL_X_MAX - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (ACCEL_Y_MAX - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (ACCEL_Z_MAX - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET ((MAGN_X_MIN + MAGN_X_MAX) / 2.0f)
#define MAGN_Y_OFFSET ((MAGN_Y_MIN + MAGN_Y_MAX) / 2.0f)
#define MAGN_Z_OFFSET ((MAGN_Z_MIN + MAGN_Z_MAX) / 2.0f)
#define MAGN_X_SCALE (100.0f / (MAGN_X_MAX - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (MAGN_Y_MAX - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (MAGN_Z_MAX - MAGN_Z_OFFSET))


// Gain for gyroscope (ITG-3200)
#define GYRO_GAIN 0.06957 // Same gain on all axes
#define GYRO_SCALED_RAD(x) (x * TO_RAD(GYRO_GAIN)) // Calculate the scaled gyro readings in radians per second

// DCM parameters
#define Kp_ROLLPITCH 0.02f
#define Ki_ROLLPITCH 0.00002f
#define Kp_YAW 1.2f
#define Ki_YAW 0.00002f

// Stuff
#define GRAVITY 256.0f // "1G reference" used for DCM filter and accelerometer calibration
#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

// Sensor variables
float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
float accel_min[3];
float accel_max[3];

float magnetom[3];
float magnetom_min[3];
float magnetom_max[3];
float magnetom_tmp[3];

float gyro[3];
float gyro_average[3];
int gyro_num_samples = 0;

// DCM variables
float MAG_Heading;
float Accel_Vector[3] = {0, 0, 0}; // Store the acceleration in a vector
float Gyro_Vector[3] = {0, 0, 0}; // Store the gyros turn rate in a vector
float Omega_Vector[3] = {0, 0, 0}; // Corrected Gyro_Vector data
float Omega_P[3] = {0, 0, 0}; // Omega Proportional correction
float Omega_I[3] = {0, 0, 0}; // Omega Integrator
float Omega[3] = {0, 0, 0};
float errorRollPitch[3] = {0, 0, 0};
float errorYaw[3] = {0, 0, 0};
float DCM_Matrix[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
float Update_Matrix[3][3] = {{0, 1, 2}, {3, 4, 5}, {6, 7, 8}};
float Temporary_Matrix[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};

// Euler angles
float yaw;
float pitch;
float roll;

// DCM timing in the main loop
unsigned long timestamp;
unsigned long timestamp_old;
float G_Dt; // Integration time for DCM algorithm

// More output-state variables
boolean output_stream_on;
boolean output_single_on;
int curr_calibration_sensor = 0;
boolean reset_calibration_session_flag = true;
int num_accel_errors = 0;
int num_magn_errors = 0;
int num_gyro_errors = 0;

Servo steeringServo;

///////////////////////////////////////////////////////////////////////////////////
//*************************************SETUP*************************************//
///////////////////////////////////////////////////////////////////////////////////
void setup()
{
  Serial.begin(57600);
  //wdt_enable(WDTO_1S);
  pingTimer[0] = millis() + 100;           // First ping starts at 100ms, gives time for the Arduino to chill before starting.
  for (uint8_t i = 1; i < SONAR_NUM; i++) // Set the starting time for each sensor.
    pingTimer[i] = pingTimer[i - 1] + PING_INTERVAL;

  delay(50);  // Give sensors enough time to start
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();

  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
  reset_sensor_fusion();

  //Setup for output pins
  //TODO remove the pin numbers
  pinMode(TEST_LED_PIN, OUTPUT);//test
  pinMode(REVERSE_PIN, OUTPUT);//reverse
  pinMode(FORWARD_PIN, OUTPUT);//forward
  pinMode(STEERING_SERVO_PIN, OUTPUT);//steering servo
  pinMode(BATTERY_MONITOR_PIN, INPUT);
  steeringServo.attach(STEERING_SERVO_PIN);
  steeringServo.write(75);
  delay(1000);
}

///////////////////////////////////////////////////////////////////////////////////
//**************************************LOOP**************************************//
///////////////////////////////////////////////////////////////////////////////////
//long last = 0;
void loop()
{
  //wdt_reset();

  if ((millis() - timestamp) >= OUTPUT__DATA_INTERVAL)
  {
    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;
    Serial.println(poll());
    //    poll();
    //    Serial.println(millis()-last);
    //    last = millis();
  }

  while (Serial.available() > 0)
  {
    char recieved = Serial.read();
    String inData = "";
    Serial.println("I have recieved!");
    if (recieved == 'c')
    {
      // Process message when new line character is recieved
      while (recieved != ',' && recieved != '\n')
      {
        recieved = (char) Serial.read();
        if (recieved != (char) - 1)
          inData += recieved;
      }
      int steeringDirection = inData.toInt();
      //      Serial.println(steeringDirection);

      inData = "";
      recieved = 'c';
      while (recieved != ',' && recieved != '\n')
      {
        recieved = (char) Serial.read();
        if (recieved != (char) - 1)
          inData += recieved;
      }
      int drivePower = inData.toInt();

      inData = "";
      recieved = 'c';
      while (recieved != ',' && recieved != '\n')
      {
        recieved = (char) Serial.read();
        if (recieved != (char) - 1)
          inData += recieved;
      }

      switch (inData.toInt())
      {
        case 0:
          break;
        case 1:
          stopMove();
          break;
      }
      inData = "";

      instruct(steeringDirection, drivePower);
      serialFlush();

    }

  }

}

///////////////////////////////////////////////////////////////////////////////////
//******************************NECESSARY FUNCTIONS******************************//
///////////////////////////////////////////////////////////////////////////////////
//Clear the incoming buffer
void serialFlush() {
  while (Serial.available() > 0) {
    char t = Serial.read();
  }
}
float oldAccel[2] = {0, 0}; //x,y
String poll()
{
  ultra();
  while (front == 0 || left == 0 || right == 0 || rear == 0)
  {
    ultra();
  }

  updateBatteryLevel();
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
  // Apply sensor calibration
  compensate_sensor_errors();
  // Run DCM algorithm
  Compass_Heading(); // Calculate magnetic heading
  Matrix_update();
  Normalize();
  Drift_correction();
  Euler_angles();
  byte timeBetweenPoll = millis() - lastPoll;
  lastPoll = millis();
  String msg;
  msg.concat(front);
  msg.concat(",");
  msg.concat(left);
  msg.concat(",");
  msg.concat(right);
  msg.concat(",");
  msg.concat(rear);
  msg.concat(",");
  msg.concat(MAG_Heading);
  msg.concat(",");
  msg.concat(yaw);
  msg.concat(",");
  msg.concat(timeBetweenPoll);
  msg.concat(",");
  msg.concat(oldAccel[0]);
  msg.concat(",");
  msg.concat(oldAccel[1]);
  msg.concat(",");
  msg.concat(accel[0]);
  msg.concat(",");
  msg.concat(accel[1]);
  msg.concat(",");
  msg.concat(batteryLevel);
  oldAccel[0] = accel[0];
  oldAccel[1] = accel[1];
  return msg;
}

void instruct(int dir, int power)//direction (-10 to 10) power 0 to 100
{
  if (power > 0)
    forward(map(power, 0, 100, 90, 255));
  else if (power == 0)
    forward(0);
  else
    reverse(map(abs(power), 0, 100, 90, 255));
  steeringServo.write(map(dir, -10, 10, 85, 65));
}

void updateBatteryLevel()
{
  batteryLevel = map(analogRead(BATTERY_MONITOR_PIN), 0, 1024, 0, 1024);
}

void software_Reset() // Restarts program from beginning but does not reset the peripherals and registers
{
asm volatile ("  jmp 0");  
} 


