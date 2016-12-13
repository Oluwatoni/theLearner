#include "Arduino.h"
#ifndef IMU_h
#define IMU_h
#include <Wire.h>

#define ACCEL_ADDRESS ((int) 0x53) 
#define MAGN_ADDRESS  ((int) 0x1E) 
#define GYRO_ADDRESS  ((int) 0x68)

// Arduino backward compatibility macros
#if ARDUINO >= 100
  #define WIRE_SEND(b) Wire.write((byte) b) 
  #define WIRE_RECEIVE() Wire.read() 
#else
  #define WIRE_SEND(b) Wire.send(b)
  #define WIRE_RECEIVE() Wire.receive() 
#endif


// When set to true, gyro drift correction will not be applied
#define DEBUG__NO_DRIFT_CORRECTION false
// Print elapsed time after each I/O loop
#define DEBUG__PRINT_LOOP_TIME false

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

 
class IMU
{
private:
	// If set true, an error message will be output if we fail to read sensor data.
	// Message format: "!ERR: reading <sensor>", followed by "\r\n".
	boolean output_errors = false;  // true or false
	float update_frequency;
       const float magn_ellipsoid_center[3] = {43.2736, 80.8049, 12.2232};
       const float magn_ellipsoid_transform[3][3] = {{0.932409, -0.0132827, -0.00197437}, {-0.0132827, 0.920629, -0.000932926}, {-0.00197437, -0.000932926, 0.999938}};	// Sensor variables
	float accel[3];  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
	float accel_min[3];
	float accel_max[3];
	float accel_offset[3];
	float accel_scale[3];
	float magnetom[3];
	float magnetom_tmp[3];
	float gyro[3];
	float gyro_average[3];
	int gyro_num_samples = 0;
	// DCM variables
	float MAG_Heading;
	float Accel_Vector[3]= {0, 0, 0}; // Store the acceleration in a vector
	float Gyro_Vector[3]= {0, 0, 0}; // Store the gyros turn rate in a vector
	float Omega_Vector[3]= {0, 0, 0}; // Corrected Gyro_Vector data
	float Omega_P[3]= {0, 0, 0}; // Omega Proportional correction
	float Omega_I[3]= {0, 0, 0}; // Omega Integrator
	float Omega[3]= {0, 0, 0};
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

public:
	IMU(float frequency);
	void Setup();
	void UpdateIMU();
	void I2C_Init();
	void Accel_Init();
	void Read_Accel();
	void Magn_Init();
	void Read_Magn();
	void Gyro_Init();
	void Read_Gyro();
	void Compass_Heading();
	void Normalize();
	void Drift_correction();
	void Matrix_update();
	void Euler_angles();
	float Vector_Dot_Product(const float v1[3], const float v2[3]);
	void Vector_Cross_Product(float out[3], const float v1[3], const float v2[3]);
	void Vector_Scale(float out[3], const float v[3], float scale);
	void Vector_Add(float out[3], const float v1[3], const float v2[3]);
	void Matrix_Multiply(const float a[3][3], const float b[3][3], float out[3][3]);
	void Matrix_Vector_Multiply(const float a[3][3], const float b[3], float out[3]);
	void init_rotation_matrix(float m[3][3], float yaw, float pitch, float roll);
	void read_sensors();
	void reset_sensor_fusion();
	void compensate_sensor_errors();
	//getters
	float GetAccX();
	float GetAccY();
	float GetAccZ();
	float GetGyroX();
	float GetGyroY();
	float GetGyroZ();
	float GetMagX();
	float GetMagY();
	float GetMagZ();
	float GetHeading();
	float GetYaw();
	float GetPitch();
	float GetRoll();
};

#endif
