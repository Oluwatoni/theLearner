import serial
import signal
import time
import threading
import rospy
import numpy as np
import tf
from tf.transformations import quaternion_from_euler
from numpy import zeros
from sensor_msgs.msg import Imu, MagneticField, Range
from std_msgs.msg import Int16, Int32, Float32, Time
from geometry_msgs.msg import AccelWithCovarianceStamped
from the_learner.msg import RawSpeedEncoder
from math import pi, radians
from threading import Thread

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
        self._br = tf.TransformBroadcaster()
        self._imu_accel_factor = 9.806 / 256.0
        self._seq = 0
        self._running = 1
        self._sensorMsg = ""
        self._sensorReadings = []
        self._ultrasonicData = zeros(7)
        self._imu_msg = Imu()
        self._mag_msg = MagneticField()
        self._acc_msg = AccelWithCovarianceStamped()
        self._enc_msg = RawSpeedEncoder()#vehicle speed
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
        rospy.Subscriber("learner/command", Int32, self.vehicle_control) 

        for i in range(7):
            self._ultrasonic_pub.append(rospy.Publisher('sensors/ultrasonic_'+str(i), Range, queue_size = 1))
        self._battery_pub = rospy.Publisher('sensors/battery_level', Int16, queue_size = 1)
        self._imu_pub = rospy.Publisher('imu/data_raw', Imu, queue_size = 1)
        self._imu_msg.header.frame_id = 'imu_link'
        self._imu_msg.orientation_covariance = [ 0.0025 , 0 , 0,
                                                 0, 0.0025, 0,
                                                 0, 0, 0.0025 ]
        self._imu_msg.angular_velocity_covariance = [ 0.02, 0 , 0,
                                                      0 , 0.02, 0,
                                                      0 , 0 , 0.02 ]
        self._imu_msg.linear_acceleration_covariance = [ 0.04 , 0 , 0,
                                                         0 , 0.04, 0,
                                                         0 , 0 , 0.04 ]
        self._mag_pub = rospy.Publisher('imu/mag', MagneticField, queue_size = 1)
        self._mag_msg.header.frame_id = 'imu_link'
        self._mag_msg.magnetic_field_covariance = [ 0.0025 , 0 , 0,
                                                    0, 0.0025, 0,
                                                    0, 0, 0.0025 ]
        self._acc_pub = rospy.Publisher('sensors/accelerometer', AccelWithCovarianceStamped,queue_size = 1)
        self._acc_msg.header.frame_id = 'acc_link'
        self._acc_msg.accel.covariance = [0.02, 0, 0, 0, 0, 0,
                                          0, 0.02, 0, 0, 0, 0,
                                          0, 0, 0.02, 0, 0, 0,
                                          0, 0, 0, 0, 0, 0,
                                          0, 0, 0, 0, 0, 0,
                                          0, 0, 0, 0, 0, 0]
        self._enc_pub = rospy.Publisher('sensors/encoder', RawSpeedEncoder, queue_size = 1)
        self._old_command = ""

    def run(self):
        while self._running:
            #if data is available in the serial buffer    
            try:            
                data_available = self._serial_port.in_waiting != 0
            except IOError:
                if self._running:
                    self._serial_port.close()
                    self._serial_port.open()
                    print "reopening serial port"
                continue
                
            if data_available:
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
                    if self._sensorReadings[0] == 'a':
                        self.publish_accelerometer()
                    elif self._sensorReadings[0] == 'e':
                        self.publish_encoder()
                    elif self._sensorReadings[0] == 'i':
                        self.publish_imu()
                    elif self._sensorReadings[0] == 't':
                        self.synchronize_time()
                    elif self._sensorReadings[0] == 'u':
                        order_u = [5,0,3]
                        rec_time = [self._sensorReadings[4], self._sensorReadings[5]]
                        self.publish_ultrasonic(order_u,rec_time)
                    elif self._sensorReadings[0] == 'v':
                        order_v = [4,1]
                        rec_time =  [self._sensorReadings[3], self._sensorReadings[4]]
                        self.publish_ultrasonic(order_v, rec_time)
                    elif self._sensorReadings[0] == 'w':
                        order_w = [6,2]
                        rec_time =  [self._sensorReadings[3], self._sensorReadings[4]]
                        self.publish_ultrasonic(order_w, rec_time)
                    else:
                        print "Invalid"
                        print self._sensorReadings[0]
            else:
                pass
    
    def publish_accelerometer(self):
        self._acc_msg.accel.accel.linear.x = (float(self._sensorReadings[1]) - 577) * 0.0817 
        self._acc_msg.accel.accel.linear.y = (float(self._sensorReadings[2]) - 560) * 0.0853 
        self._acc_msg.header.stamp.secs = int(self._sensorReadings[3])
        self._acc_msg.header.stamp.nsecs = int(self._sensorReadings[4]) * 1000
        self._seq += 1
        self._acc_msg.header.seq = self._seq
        self._acc_pub.publish(self._acc_msg)
        self._br.sendTransform((0.04, 0.0, 0.072),
                         quaternion_from_euler((-pi/2),0,0),
                         rospy.Time.now(),
                         "acc_link",
                         "learner/base_link")

    def publish_encoder(self):
        self._enc_msg.speed = float(self._sensorReadings[1])
        self._enc_msg.header.stamp.secs = int(self._sensorReadings[2])
        self._enc_msg.header.stamp.nsecs = int(self._sensorReadings[3]) * 1000 
        self._seq += 1
        self._enc_msg.header.seq = self._seq
        self._enc_pub.publish(self._enc_msg)

    def publish_imu(self):
        yaw =  -1 * float(self._sensorReadings[1])
        pitch = float(self._sensorReadings[2])
        roll = float(self._sensorReadings[3])
        self._imu_msg.linear_acceleration.x = float(self._sensorReadings[4])
        self._imu_msg.linear_acceleration.y = float(self._sensorReadings[5])
        self._imu_msg.linear_acceleration.z = float(self._sensorReadings[6])
        self._imu_msg.angular_velocity.x = float(self._sensorReadings[7])# * radians(0.06957) #gyro gain
        self._imu_msg.angular_velocity.y = float(self._sensorReadings[8])# * radians(0.06957)
        self._imu_msg.angular_velocity.z = float(self._sensorReadings[9])# * radians(0.06957)
        q = quaternion_from_euler(roll,pitch,yaw)
        self._imu_msg.orientation.x = q[0]
        self._imu_msg.orientation.y = q[1]
        self._imu_msg.orientation.z = q[2]
        self._imu_msg.orientation.w = q[3]
#        self._mag_msg.magnetic_field.x = (float(self._sensorReadings[7]) * 0.92) / 10000000.0
#        self._mag_msg.magnetic_field.y = (float(self._sensorReadings[8]) * 0.92) / 10000000.0
#        self._mag_msg.magnetic_field.z = (float(self._sensorReadings[9]) * 0.92) / 10000000.0
        self._imu_msg.header.stamp.secs = int(self._sensorReadings[11])
        self._imu_msg.header.stamp.nsecs = int(self._sensorReadings[12]) * 1000 
#        self._mag_msg.header.stamp.secs = int(self._sensorReadings[11])
#        self._mag_msg.header.stamp.nsecs = int(self._sensorReadings[12]) * 1000 
        self._seq += 1
        self._imu_msg.header.seq = self._seq
#        self._mag_msg.header.seq = self._seq
        self._imu_pub.publish(self._imu_msg)
#        self._mag_pub.publish(self._mag_msg)
        self._batteryLevels = np.roll(self._batteryLevels,-1)
        self._batteryLevels[19] = int(self._sensorReadings[10])
        if self._batteryLevels[0] != 0:
            self._battery_pub.publish(np.median(self._batteryLevels))
        self._br.sendTransform((0.04, 0.0, 0.152),
                          quaternion_from_euler(0,0,0),
                          rospy.Time.now(),
                          "imu_link",
                          "learner/base_link")

    #publishes ultrasonic data in the order supplied
    def publish_ultrasonic(self, order, time):
        for i in range(len(order)):
            self._ultrasonic_msg.range = float(self._sensorReadings[i+1]) / 100.0
            self._ultrasonic_msg.header.frame_id = "ultrasonic_" + str(order[i]) + "_link"
            self._ultrasonic_msg.header.stamp.secs = int(time[0])
            self._ultrasonic_msg.header.stamp.nsecs = int(time[1]) * 1000
            self._seq += 1
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

    def vehicle_control(self, data):
        if self._old_command != data.data:
            msg = "r,"    #reset message
            # set steering angle
            msg += str(int((data.data >> 1) & 127) - 30)
            msg += ","
            # set throttle
            msg += str(int(data.data >> 10))
            msg += ","
            #set brake
            msg += str(data.data & 1)
            msg += ","
            msg += str(generate_checksum(msg.split(","),False))
            msg += "\n" 
            self._serial_port.write(msg)
        self._old_command = data.data

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
