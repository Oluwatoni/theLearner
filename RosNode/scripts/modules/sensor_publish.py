from threading import Thread
import serial
import signal
import time
import threading
import rospy
import numpy as np
from tf.transformations import quaternion_from_euler
from numpy import zeros
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from std_msgs.msg import Int16, Float32, Time
#from geometry.msg import AccelWithCovarianceStamped

#thread that handles incoming messages from the arduino
class ArduinoMonitor (Thread):
    def __init__(self, threadID, serial_port):
        print "Starting Arduino Monitor"
        Thread.__init__(self)
        #setup serial port
        self._serial_port = serial.Serial(serial_port,115200)
        self._serial_port.timeout = 0.5
        self._serial_port.xonxoff = 0
        self._serial_port.rtscts = 0
        self._threadID = threadID
        self._imu_accel_factor = 9.806 / 256.0
        self._seq = 0
        self._running = 1
        self._sensorMsg = ""
        self._sensorReadings = []
        self._ultrasonicData = zeros(7)
        self._imu_msg = Imu()#Yaw,Pitch,Roll,Accel x,y,z,gyro x,y,z
#        self._accelerometer = AccelWithCovarianceStamped()#Accx, Accy
        self._batteryLevels = zeros(20)
        self._batteryLevel = 0
        self._robot_time = 0
        self._ultrasonic_pub = []
        self._ultrasonic_msg = Range()
        self._ultrasonic_msg.range = 0.0
        self._ultrasonic_msg.field_of_view = 0.5236
        self._ultrasonic_msg.radiation_type = self._ultrasonic_msg.ULTRASOUND
        self._ultrasonic_msg.min_range = 0.0
        self._ultrasonic_msg.max_range = 2.5

        for i in range(7):
            self._ultrasonic_pub.append(rospy.Publisher('arduino_sensors/ultrasonic_'+str(i),Range,queue_size = 1))
        self._battery_pub = rospy.Publisher('arduino_sensors/battery_level', Int16,queue_size = 1)
        self._imu_pub = rospy.Publisher('arduino_sensors/imu',Imu,queue_size = 1)
        self._imu_msg.header.frame_id = 'learner_imu_link'
        self._imu_msg.orientation_covariance = [ 0.0025 , 0 , 0,
                                                 0, 0.0025, 0,
                                                 0, 0, 0.0025 ]
        self._imu_msg.angular_velocity_covariance = [ 0.02, 0 , 0,
                                                      0 , 0.02, 0,
                                                      0 , 0 , 0.02 ]
        self._imu_msg.linear_acceleration_covariance = [ 0.04 , 0 , 0,
                                                         0 , 0.04, 0,
                                                         0 , 0 , 0.04 ]

    def run(self):
        while self._running:
            #if data is available in the serial buffer    
            if self._serial_port.in_waiting != 0:

                #read a line
                self._sensorMsg = self._serial_port.readline()
                self._sensorReadings = self._sensorMsg.split(',')
                #print self._sensorReadings

                #checksum check
                if not(self.data_is_valid(self._sensorReadings)):
                    continue #wait for the next message to come in
                #valid message
                try:
                    self._sensorReadings[0]
                except IndexError:
                    continue
                    
                else:
                    #publish second Accelerometer
                    if self._sensorReadings[0] == 'a':
                        pass
                    #Publish IMU
                    elif self._sensorReadings[0] == 'i':
                        publish_imu()
                    #send the current time to synchronize
                    elif self._sensorReadings[0] == 't':
                        synchronize_time()
                   #publish Ultrasonic sensors 1,4,6
                    elif self._sensorReadings[0] == 'u':
                        order_u = [5,0,3]
                        rec_time = [self._sensorReadings[4], self._sensorReadings[5]]
                        self.publish_ultrasonic(order_u,rec_time)
                        #print "Ultra1"
                    #publish Ultrasonic sensors 2,5
                    elif self._sensorReadings[0] == 'v':
                        order_v = [4,1]
                        rec_time =  [self._sensorReadings[3], self._sensorReadings[4]]
                        self.publish_ultrasonic(order_v, rec_time)
                        #print "Ultra2"
                    #publish Ultrasonic sensors 3,7
                    elif self._sensorReadings[0] == 'w':
                        order_w = [6,2]
                        rec_time =  [self._sensorReadings[3], self._sensorReadings[4]]
                        self.publish_ultrasonic(order_w, rec_time)
                        #print "Ultra3"
                    else:
                        print "Invalid"
            else:
                pass

    def publish_imu(self):
            yaw =  float(self._sensorReadings[1])
            pitch = float(self._sensorReadings[2])
            roll = float(self._sensorReadings[3])
            self._imu_msg.linear_acceleration.x = -float(self._sensorReadings[4]) * self._imu_accel_factor
            self._imu_msg.linear_acceleration.y = float(self._sensorReadings[5]) * self._imu_accel_factor
            self._imu_msg.linear_acceleration.z = float(self._sensorReadings[6]) * self._imu_accel_factor
            self._imu_msg.angular_velocity.x = float(self._sensorReadings[7])
            self._imu_msg.angular_velocity.y = -float(self._sensorReadings[8])
            self._imu_msg.angular_velocity.z = -float(self._sensorReadings[9])
            q = quaternion_from_euler(roll,pitch,yaw)
            self._imu_msg.orientation.x = q[0]
            self._imu_msg.orientation.y = q[1]
            self._imu_msg.orientation.z = q[2]
            self._imu_msg.orientation.w = q[3]
            self._imu_msg.header.stamp.secs = int(self._sensorReadings[11])
            self._imu_msg.header.stamp.nsecs = int(self._sensorReadings[12]) * 1000 
            self._imu_msg.header.seq = self._seq
            self._seq += 1
            self._imu_pub.publish(self._imu_msg)
            self._batteryLevels = np.roll(self._batteryLevels,-1)
            self._batteryLevels[19] = int(self._sensorReadings[10])
            if self._batteryLevels[0] != 0:
                self._battery_pub.publish(self._batteryLevels.median())
      
    #publishes ultrasonic data in the order supplied
    def publish_ultrasonic(self, order, time):
        for i in range(len(order)):
            self._ultrasonic_msg.range = float(self._sensorReadings[i+1]) / 100.0
            self._ultrasonic_msg.header.frame_id = "learner_ultrasonic_" + str(order[i]) + "_link"
            self._ultrasonic_msg.header.stamp.secs = int(time[0])
            self._ultrasonic_msg.header.stamp.nsecs = int(time[1]) * 1000
            self._ultrasonic_msg.header.seq = self._seq
            self._ultrasonic_pub[order[i]].publish(self._ultrasonic_msg)
            
    def synchronize_time(self):
        time_msg =  "t,"
        now = rospy.Time.now()
        time_msg += (str(now.secs) + ",")
        time_msg += (str(now.nsecs/1000) + ",")
        time_msg += str(generate_checksum(time_msg.split(","),False))
        time_msg += "\n"
        self._serial_port.write(time_msg); 


    #validates message using the checksum
    def data_is_valid(self, data):
        try:
            checksum = int(data.pop())
        except ValueError:
            return False
        if generate_checksum(data, True)  == checksum:
            #print "valid data"
            return True
        else:
            print "invalid data"
            print checksum
            print generate_checksum(data,True)
            print data
            return False

    #stops the thread
    def stop(self):
        print "Im stopping!"
        self._serial_port.close()
        self._running = 0
        
def generate_checksum(data, check_if_number):
    sumOfBytes = 0
    for i in range(len(data)):
        for char in data[i]:
            sumOfBytes += ord(char)
        sumOfBytes += ord(",")
    return (sumOfBytes % 255)



