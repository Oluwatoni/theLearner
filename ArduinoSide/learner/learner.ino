/*
  Oluwatoni Ogunmade
  funmitoni2@yahoo.co.uk

  inspiration below.
  http://projectsfromtech.blogspot.com/2014/01/i2c-hc-sr04-sonar-module-attiny85-i2c.html
*/
#include <Wire.h>
#include <NewPing.h>
#include <Servo.h>
//#include <TinyGPS++.h>
//#include <SoftwareSerial.h>
//#define DEBUG
//defines for the ultrasonic sensors
#define MCU1_I2C 1
#define MCU2_I2C 2
#define SONAR_NUM 3
#define MAX_DISTANCE 250 //in cm
#define FORWARD_PIN 5
#define REVERSE_PIN 6
#define STEERING_SERVO_PIN 3
#define TEST_LED_PIN 13
#define BATTERY_MONITOR_PIN A7
#define ULTRASONIC_DELAY 16
byte Distance[7] = {};                            // Where the range data is stored

bool direction;
int batteryLevel;

NewPing sonar[SONAR_NUM] =       // Sensor object array.
{
  NewPing(7, 8, MAX_DISTANCE), //ultra 5 // Each sensor's trigger pin, echo pin, and max distance to ping.
  NewPing(A2, A3, MAX_DISTANCE),//ultra 7
  NewPing(4, 2, MAX_DISTANCE)//ultra 6
};

Servo steeringServo;

// #define RX_PIN A1
// #define TX_PIN A0
// #define GPS_BAUD 9600

// The TinyGPS++ object
//TinyGPSPlus gps;
//SoftwareSerial ss(RX_PIN, TX_PIN);

//IMU defines
#define HW__VERSION_CODE 10724 // SparkFun "9DOF Sensor Stick" version "SEN-10724" (HMC5883L magnetometer)

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT__DATA_INTERVAL 20  // in milliseconds

// If set true, an error message will be output if we fail to read sensor data.
// Message format: "!ERR: reading <sensor>", followed by "\r\n".
boolean output_errors = false;  // true or false

// SENSOR CALIBRATION
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
                                              { -0.0348895, 0.0566169, 0.962865}};

// Gyroscope
// "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
#define GYRO_AVERAGE_OFFSET_X ((float) -42.88)
#define GYRO_AVERAGE_OFFSET_Y ((float) -29.52)
#define GYRO_AVERAGE_OFFSET_Z ((float) 7.08)


// DEBUG OPTIONS

// When set to true, gyro drift correction will not be applied
#define DEBUG__NO_DRIFT_CORRECTION false
// Print elapsed time after each I/O loop
#define DEBUG__PRINT_LOOP_TIME false


// Check if hardware version code is defined
#ifndef HW__VERSION_CODE
// Generate compile error
#error YOU HAVE TO SELECT THE HARDWARE YOU ARE USING! See "HARDWARE OPTIONS" in "USER SETUP AREA" at top of Razor_AHRS.ino!
#endif

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
#define STATUS_LED_PIN 13  // Pin number of status LED
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

void read_sensors() {
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?

void reset_sensor_fusion() {
  float temp1[3];
  float temp2[3];
  float xAxis[] = {1.0f, 0.0f, 0.0f};

  read_sensors();
  timestamp = millis();

  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  pitch = -atan2(accel[0], sqrt(accel[1] * accel[1] + accel[2] * accel[2]));

  // GET ROLL
  // Compensate pitch of gravity vector
  Vector_Cross_Product(temp1, accel, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  roll = atan2(temp2[1], temp2[2]);

  // GET YAW
  Compass_Heading();
  yaw = MAG_Heading;

  // Init rotation matrix
  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

// Apply calibration to raw sensor readings
void compensate_sensor_errors() {
  // Compensate accelerometer error
  accel[0] = (accel[0] - ACCEL_X_OFFSET) * ACCEL_X_SCALE;
  accel[1] = (accel[1] - ACCEL_Y_OFFSET) * ACCEL_Y_SCALE;
  accel[2] = (accel[2] - ACCEL_Z_OFFSET) * ACCEL_Z_SCALE;

  // Compensate magnetometer error
#if CALIBRATION__MAGN_USE_EXTENDED == true
  for (int i = 0; i < 3; i++)
    magnetom_tmp[i] = magnetom[i] - magn_ellipsoid_center[i];
  Matrix_Vector_Multiply(magn_ellipsoid_transform, magnetom_tmp, magnetom);
#else
  magnetom[0] = (magnetom[0] - MAGN_X_OFFSET) * MAGN_X_SCALE;
  magnetom[1] = (magnetom[1] - MAGN_Y_OFFSET) * MAGN_Y_SCALE;
  magnetom[2] = (magnetom[2] - MAGN_Z_OFFSET) * MAGN_Z_SCALE;
#endif

  // Compensate gyroscope error
  gyro[0] -= GYRO_AVERAGE_OFFSET_X;
  gyro[1] -= GYRO_AVERAGE_OFFSET_Y;
  gyro[2] -= GYRO_AVERAGE_OFFSET_Z;
}

// Reset calibration session if reset_calibration_session_flag is set
void check_reset_calibration_session()
{
  // Raw sensor values have to be read already, but no error compensation applied

  // Reset this calibration session?
  if (!reset_calibration_session_flag) return;

  // Reset acc and mag calibration variables
  for (int i = 0; i < 3; i++) {
    accel_min[i] = accel_max[i] = accel[i];
    magnetom_min[i] = magnetom_max[i] = magnetom[i];
  }

  // Reset gyro calibration variables
  gyro_num_samples = 0;  // Reset gyro calibration averaging
  gyro_average[0] = gyro_average[1] = gyro_average[2] = 0.0f;

  reset_calibration_session_flag = false;
}

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

void setup()
{
  Wire.begin();

  // Init sensors
  delay(100);  // Give sensors enough time to start
  I2C_Init();
  Accel_Init();
  Magn_Init();
  Gyro_Init();

  // Read sensors, init DCM algorithm
  delay(20);  // Give sensors enough time to collect data
  reset_sensor_fusion();

  Serial.begin(115200);
  //  ss.begin(GPS_BAUD);

  pinMode(TEST_LED_PIN, OUTPUT);//test
  pinMode(REVERSE_PIN, OUTPUT);//reverse
  pinMode(FORWARD_PIN, OUTPUT);//forward
  pinMode(STEERING_SERVO_PIN, OUTPUT);//steering servo
  pinMode(BATTERY_MONITOR_PIN, INPUT);
  steeringServo.attach(STEERING_SERVO_PIN);
  steeringServo.write(75);
  delay(1000);
}

unsigned long temp = 0, now  =0;
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
  unsigned long startTime  = millis();
  Wire.beginTransmission(MCU1_I2C);
  Wire.write(56);                            //exotic byte to sync the communication with MCU2
  Wire.endTransmission();

  Wire.beginTransmission(MCU2_I2C);
  Wire.write(56);                            //exotic byte to sync the communication with MCU2
  Wire.endTransmission();

  //beginning of first wave
  //MCU1
  Wire.requestFrom(MCU1_I2C, 1);       // The TinyWire library only allows for one byte to be requested at a time
  while (Wire.available() == 0)  ;
  Wire.read();
  //MCU2
  Wire.requestFrom(MCU2_I2C, 1);       // The TinyWire library only allows for one byte to be requested at a time
  while (Wire.available() == 0)  ;
  Wire.read();

  temp = millis();
  Distance[5] = sonar[2].ping_cm();//ultrasonic sensor 6
  now  = millis();
  if ((now - temp ) < ULTRASONIC_DELAY && (int)(now - temp) > 0)
  {
    delay((temp + ULTRASONIC_DELAY) - now); //make sure 15ms has elapsed before retrieving range data //TODO make define
  }
  
  //MCU1
  Wire.requestFrom(MCU1_I2C, 1);
  while (Wire.available() == 0)  ;
  Distance[0] = Wire.read();

  //MCU2
  Wire.requestFrom(MCU2_I2C, 1);
  while (Wire.available() == 0)  ;
  Distance[3] = Wire.read();
  //end of first wave of sensor data

  delay(25);
  //beginning of second wave
  //MCU1
  Wire.requestFrom(MCU1_I2C, 1);
  while (Wire.available() == 0)  ;
  Wire.read();

  temp = millis();
  Distance[4] = sonar[0].ping_cm();// ultrasonic sensor 5
  now  = millis();
  if ((now - temp) < ULTRASONIC_DELAY && (int)(now - temp) > 0)
  {
    delay((temp + ULTRASONIC_DELAY) - now);
  }

  //MCU1
  Wire.requestFrom(MCU1_I2C, 1);
  while (Wire.available() == 0)  ;
  Distance[1] = Wire.read();
  //end of second wave of sensor data

  //get battery Level
  updateBatteryLevel();
  delay(25);
  //beginning of third wave
  //MCU2
  Wire.requestFrom(MCU2_I2C, 1);
  while (Wire.available() == 0)  ;
  Wire.read();

  temp = millis();
  Distance[6] = sonar[1].ping_cm();//ultrasonic sensor 7
  now = millis();
  if ((now - temp) < ULTRASONIC_DELAY && (int)(now - temp) > 0)
    delay((temp + ULTRASONIC_DELAY) - now);
  //MCU2
  Wire.requestFrom(MCU2_I2C, 1);
  while (Wire.available() == 0)  ;
  Distance[2] = Wire.read();
  //end of third wave of sensor data

  

  // Takes 6-8ms
  if ((millis() - timestamp) >= OUTPUT__DATA_INTERVAL)
  {
    //    startTime = millis();
    timestamp_old = timestamp;
    timestamp = millis();
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else G_Dt = 0;

    // Update sensor readings
    read_sensors();

    // Apply sensor calibration
    compensate_sensor_errors();

    // Run DCM algorithm
    Compass_Heading(); // Calculate magnetic heading
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();
  }
  
  delay(25);
  float freq = 1.0 / ((millis() - startTime) / 1000.0);
  String msg;
  msg.concat(Distance[0]);
  msg.concat(",");
  msg.concat(Distance[1]);
  msg.concat(",");
  msg.concat(Distance[2]);
  msg.concat(",");
  msg.concat(Distance[3]);
  msg.concat(",");
  msg.concat(Distance[4]);
  msg.concat(",");
  msg.concat(Distance[5]);
  msg.concat(",");
  msg.concat(Distance[6]);
  msg.concat(",");
  msg.concat(accel[0]);
  msg.concat(",");
  msg.concat(accel[1]);
  msg.concat(",");
  msg.concat(accel[2]);
  msg.concat(",");
  msg.concat(magnetom[0]);
  msg.concat(",");
  msg.concat(magnetom[1]);
  msg.concat(",");
  msg.concat(magnetom[2]);
  msg.concat(",");
  msg.concat(gyro[0]);
  msg.concat(",");
  msg.concat(gyro[1]);
  msg.concat(",");
  msg.concat(gyro[2]);
  msg.concat(",");
  msg.concat(batteryLevel);
  msg.concat(",");
  msg.concat(freq);
  msg.concat(",");
  msg.concat(millis());
  msg.concat(",");
  char buffer[msg.length()];
  msg.toCharArray(buffer, msg.length());
  msg.concat(generateChecksum(buffer, msg.length()));
  Serial.println(msg);
}

//handles RC msgs
void serialEvent() {
  int steering, throttle, stopRequested;
  inputString = "";
    delay(10);
    inputString = Serial.readStringUntil(',');
    if (inputString.startsWith("r"))
    {
      steering = (inputString.substring(1)).toFloat();
      inputString = Serial.readStringUntil(',');
      throttle = inputString.toFloat();
      inputString = Serial.readStringUntil('\n');
      stopRequested = inputString.toFloat();
#ifdef DEBUG
      digitalWrite(13, HIGH);
      String debug_msg = "";
      debug_msg.concat(steering);
      debug_msg.concat(",");
      debug_msg.concat(throttle);
      Serial.println(debug_msg);
#endif
    }
    instruct(steering,throttle);
}


