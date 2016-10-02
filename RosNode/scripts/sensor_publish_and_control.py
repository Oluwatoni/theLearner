#!/usr/bin/env python
import sys
import serial
import signal
import time
import threading
import rospy
import numpy as np
from tf.transformations import quaternion_from_euler
from numpy import zeros
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from std_msgs.msg import Int16, Float32, Time

time.sleep(5)
#read in serial port
serial_port = sys.argv[1]

#setup serial port
_arduino_serial_port = serial.Serial(serial_port,115200)
_arduino_serial_port.timeout = None

def generate_checksum(data, check_if_number):
    sumOfBytes = 0
    for i in range(len(data)):
        for char in data[i]:
            sumOfBytes += ord(char)
        sumOfBytes += ord(",")
    return (sumOfBytes % 255)


class PS3Controller:
    def __init__(self):
        "PS3 controller node constructor"
        #self.__estop = 11                 # emergency stop
        # tele-operation controls
        self.__steer = 0                  # steering axis
        self.__drive = 13                  # shift to Drive
        self.__reverse = 12                # shift to Reverse
        self.__brake = 13
	self.__run_suspend = 6
        self.__auto = False
        #Arduino control msg
        self.__msg = "r,"   #r indicates the start of a message
        self.__old_msg = ""
        self.__old_time = time.clock()

        # initialize ROS joystick suscriber
        self.__joy = rospy.Subscriber('joy', Joy, self.joy_callback)

    def joy_callback(self, joy):
        "invoked every time a joystick message arrives"
        #rospy.logdebug('joystick input:\n' + str(joy))
        
        if joy.buttons[self.__run_suspend]:
            self.__auto = not(self.__auto)
            rospy.logwarn('autonomous mode' + str(self.__auto))

        # set steering angle
        self.__msg+= str(int(-80 * joy.axes[self.__steer]))
        self.__msg+= ","

        # set driving speed
        self.__msg+= str(int(100 *( -joy.axes[self.__drive] + joy.axes[self.__reverse])))
        self.__msg+= ","
    
        #set brake
        if joy.buttons[self.__brake]:
            self.__msg += "1,"
        else:
            self.__msg += "0,"
            self.__msg += str(generate_checksum(self.__msg.split(","),False))
            self.__msg += "\n"
        
        freq = 20.0
        #reduce the controller frequency to 10Hz and write to the arduino
        if time.clock() - self.__old_time >= (1.0/freq):
            _arduino_serial_port.write(self.__msg)
            #_arduino_serial_port.flush()
            self.__old_time = time.clock()
            #print self.__msg
        self.__old_msg = self.__msg
        self.__msg = "r,"    #reset message

#thread that handles incoming messages from the arduino
class ArduinoMonitor (threading.Thread):
    def __init__(self, threadID, name):
        print "Starting Arduino Monitor"
        threading.Thread.__init__(self)
        self.__threadID = threadID
        self.__name = name
        self.__imu_accel_factor = 9.806 / 256.0
        self.__seq = 0
        self.__running = 1
        self.__sensorMsg = ""
        self.__sensorReadings = []
        self.__ultrasonicData = zeros(7)
        self.__imu_msg = Imu()#Yaw,Pitch,Roll,Accel x,y,z,gyro x,y,z
        self.__accelerometer = Imu()#Accx, Accy
        self.__batteryLevels = zeros(20)
        self.__batteryLevel = 0
        self.__robot_time = 0
        self.__ultrasonic_pub = []
        self.__ultrasonic_msg = Range()
        self.__ultrasonic_msg.range = 0.0
        self.__ultrasonic_msg.field_of_view = 0.5236
        self.__ultrasonic_msg.radiation_type = self.__ultrasonic_msg.ULTRASOUND
        self.__ultrasonic_msg.min_range = 0.0
        self.__ultrasonic_msg.max_range = 2.5

        for i in range(7):
            self.__ultrasonic_pub.append(rospy.Publisher('arduino_sensors/ultrasonic_'+str(i),Range,queue_size = 1))
        self.__battery_pub = rospy.Publisher('arduino_sensors/battery_level', Int16,queue_size = 1)
        self.__imu_pub = rospy.Publisher('arduino_sensors/imu',Imu,queue_size = 1)
        self.__imu_msg.header.frame_id = 'learner_imu_link'
        self.__imu_msg.orientation_covariance = [ 0.0025 , 0 , 0,
                                                 0, 0.0025, 0,
                                                 0, 0, 0.0025 ]
        self.__imu_msg.angular_velocity_covariance = [ 0.02, 0 , 0,
                                                      0 , 0.02, 0,
                                                      0 , 0 , 0.02 ]
        self.__imu_msg.linear_acceleration_covariance = [ 0.04 , 0 , 0,
                                                         0 , 0.04, 0,
                                                         0 , 0 , 0.04 ]

    def run(self):
        while self.__running:
            #if data is available in the serial buffer

            #try:
            if _arduino_serial_port.in_waiting != 0:
            #except IOError:
            #   continue

                #read a line
                self.__sensorMsg = _arduino_serial_port.readline()
                self.__sensorReadings = self.__sensorMsg.split(',')
                #print self.__sensorReadings

                #checksum check
                if not(self.data_is_valid(self.__sensorReadings)):
                    continue #wait for the next message to come in
                #valid message
                try:
                    self.__sensorReadings[0]
                except IndexError:
                    continue
                    
                else:
                    #publish second Accelerometer
                    if self.__sensorReadings[0] == 'a':
                        pass
                         
                    #Publish IMU
                    elif self.__sensorReadings[0] == 'i':
                        yaw =  float(self.__sensorReadings[1])
                        pitch = float(self.__sensorReadings[2])
                        roll = float(self.__sensorReadings[3])
                        self.__imu_msg.linear_acceleration.x = -float(self.__sensorReadings[4]) * self.__imu_accel_factor
                        self.__imu_msg.linear_acceleration.y = float(self.__sensorReadings[5]) * self.__imu_accel_factor
                        self.__imu_msg.linear_acceleration.z = float(self.__sensorReadings[6]) * self.__imu_accel_factor
                        self.__imu_msg.angular_velocity.x = float(self.__sensorReadings[7])
                        self.__imu_msg.angular_velocity.y = -float(self.__sensorReadings[8])
                        self.__imu_msg.angular_velocity.z = -float(self.__sensorReadings[9])
                        q = quaternion_from_euler(roll,pitch,yaw)
                        self.__imu_msg.orientation.x = q[0]
                        self.__imu_msg.orientation.y = q[1]
                        self.__imu_msg.orientation.z = q[2]
                        self.__imu_msg.orientation.w = q[3]
                        #print "Time"
                        #print int(str(rospy.Time.now()))- int(self.__sensorReadings[11] + self.__sensorReadings[12] + "000")
                        self.__imu_msg.header.stamp.secs = int(self.__sensorReadings[11])
                        self.__imu_msg.header.stamp.nsecs = int(self.__sensorReadings[12]) * 1000 
                        self.__imu_msg.header.seq = self.__seq
                        self.__seq += 1
                        self.__imu_pub.publish(self.__imu_msg)
                        self.__batteryLevels = np.roll(self.__batteryLevels,-1)
                        #print self.__batteryLevels
                        self.__batteryLevels[19] = int(self.__sensorReadings[10])
                        if self.__batteryLevels[0] != 0:
                            self.__battery_pub.publish(self.__batteryLevels.mean())
                        #print "imu"
    
                    #send the current time to synchronize
                    elif self.__sensorReadings[0] == 't':
                        time_msg =  "t,"
                        now = rospy.Time.now()
                       	time_msg += (str(now.secs) + ",")
                        time_msg += (str(now.nsecs/1000) + ",")
                        time_msg += str(generate_checksum(time_msg.split(","),False))
                        time_msg += "\n"
                        #print time_msg
                        _arduino_serial_port.write(time_msg); 

                    #publish Ultrasonic sensors 1,4,6
                    elif self.__sensorReadings[0] == 'u':
                        order_u = [5,0,3]
                        rec_time = [self.__sensorReadings[4], self.__sensorReadings[5]]
                        self.publish_ultrasonic(order_u,rec_time)
                        #print "Ultra1"
                    #publish Ultrasonic sensors 2,5
                    elif self.__sensorReadings[0] == 'v':
                        order_v = [4,1]
                        rec_time =  [self.__sensorReadings[3], self.__sensorReadings[4]]
                        self.publish_ultrasonic(order_v, rec_time)
                        #print "Ultra2"
                    #publish Ultrasonic sensors 3,7
                    elif self.__sensorReadings[0] == 'w':
                        order_w = [6,2]
                        rec_time =  [self.__sensorReadings[3], self.__sensorReadings[4]]
                        self.publish_ultrasonic(order_w, rec_time)
                        #print "Ultra3"
                    else:
                        print "Invalid"
            else:
                pass

    #publishes ultrasonic data in the order supplied
    def publish_ultrasonic(self, order, time):
        for i in range(len(order)):
            self.__ultrasonic_msg.range = float(self.__sensorReadings[i+1]) / 100.0
            self.__ultrasonic_msg.header.frame_id = "learner_ultrasonic_" + str(order[i]) + "_link"
            self.__ultrasonic_msg.header.stamp.secs = int(time[0])
            self.__ultrasonic_msg.header.stamp.nsecs = int(time[1]) * 1000
            self.__ultrasonic_msg.header.seq = self.__seq
            self.__ultrasonic_pub[order[i]].publish(self.__ultrasonic_msg)
            
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
        self.__running = 0

def main():
    rospy.init_node('sensor_publish_and_control', anonymous = True)
    global learnerRead
    learnerRead = ArduinoMonitor(1, "learner")
    learnerRead.start()
    global controller
    controller = PS3Controller()
    rospy.loginfo('joystick vehicle controller starting')

    try:
        rospy.spin()
    except rospy.ROSInterruptException: pass
    rospy.loginfo('joystick vehicle controller finished')
    learnerRead.stop()

if __name__ == '__main__':
    main()

sys.exit(0)
_arduino_serial_port.close()
