#!/usr/bin/env python
import sys
import serial
import signal
import time
import threading
import rospy
from tf.transformations import quaternion_from_euler
from numpy import zeros
from sensor_msgs.msg import Joy
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from std_msgs.msg import Int16

serial_port = sys.argv[1]

_arduino_serial_port = serial.Serial(serial_port,115200)
_arduino_serial_port.timeout = None

def generate_checksum(data, check_if_number):
    sumOfBytes = 0
    for i in range(len(data)):
        if check_if_number and (i > 0):
            try:
                float(data[i])
            except ValueError:
                #print data
                return -1
        for char in data[i]:
            sumOfBytes += ord(char)
        sumOfBytes += ord(",")
    return (sumOfBytes % 255)


class PS3Controller:
    def __init__(self):
        "PS3 controller node constructor"
        #state controls
        self.__run_suspend = 10           # toggle between autonomous run and teleop
        #self.__estop = 11                 # emergency stop

        # tele-operation controls
        self.__steer = 0                  # steering axis
        self.__drive = 13                  # shift to Drive
        self.__reverse = 12                # shift to Reverse
        self.__brake = 13
        self.__auto = False
        #Arduino control msg
        self.__msg = "r" #r means rc a means autonomous and e is for e-stop
        self.__old_msg = ""
        self.__old_time = time.clock()

        # initialize ROS topics
        self.__joy = rospy.Subscriber('joy', Joy, self.joy_callback)

    def joy_callback(self, joy):
        "invoked every time a joystick message arrives"
        #rospy.logdebug('joystick input:\n' + str(joy))
        '''
        # handle E-stop buttons
         if joy.buttons[self.__estop]:
            rospy.logwarn('emergency stop')
            self.__msg = "e"
        #TODO add estop routine
        '''
        if joy.buttons[self.__run_suspend]:
            self.__auto = not(self.__auto)
            rospy.logwarn('autonomous mode' + str(self.__auto))

        # set steering angle
        self.__msg+= str(int(-100 * joy.axes[self.__steer]))
        self.__msg+= ","

        # set driving speed
        self.__msg+= str(int(50 *( -joy.axes[self.__drive] + joy.axes[self.__reverse])))
        self.__msg+= ","

        if joy.buttons[self.__brake]:
            self.__msg += "1,"
        else:
            self.__msg += "0,"
            self.__msg += str(generate_checksum(self.__msg.split(","),False))
            self.__msg += "\n"
        #print self.__msg

        if time.clock() - self.__old_time >= 0.1:
            _arduino_serial_port.write(self.__msg)
            #_arduino_serial_port.flush()
            self.__old_time = time.clock()
            print self.__msg
        self.__old_msg = self.__msg
        self.__msg = self.__msg[0]

#thread that handles incoming messages from the arduino
class ArduinoMonitor (threading.Thread):
    def __init__(self, threadID, name):
        print "Starting Arduino Monitor"
        threading.Thread.__init__(self)
        self.__threadID = threadID
        self.__name = name
        self.__accel_factor = 9.806 / 256.0
        self.__seq = 0
        self.__running = 1
        self.__sensorMsg = ""
        self.__sensorReadings = []
        self.__ultrasonicData = zeros(7)
        self.__imuMsg = Imu()#Yaw,Pitch,Roll,Accel x,y,z,gyro x,y,z
        self.__batteryLevel = 0
        self.__ultrasonic_pub = []
        self.__ultrasonic_msg = Range()
        self.__ultrasonic_msg.range = 0.0
        self.__ultrasonic_msg.field_of_view = 0.5236
        self.__ultrasonic_msg.radiation_type = self.__ultrasonic_msg.ULTRASOUND
        self.__ultrasonic_msg.min_range = 0.0
        self.__ultrasonic_msg.max_range = 2.0
        for i in range(7):
            self.__ultrasonic_pub.append(rospy.Publisher('arduino_sensors/ultrasonic_'+str(i),Range,queue_size = 1))
        self.__battery_pub = rospy.Publisher('arduino_sensors/battery_level', Int16,queue_size = 1)
        self.__imu_pub = rospy.Publisher('arduino_sensors/imu',Imu,queue_size = 1)
        self.__imuMsg.header.frame_id = 'learner_imu_link'
        self.__imuMsg.orientation_covariance = [ 0.0025 , 0 , 0,
                                                 0, 0.0025, 0,
                                                 0, 0, 0.0025 ]
        self.__imuMsg.angular_velocity_covariance = [ 0.02, 0 , 0,
                                                      0 , 0.02, 0,
                                                      0 , 0 , 0.02 ]
        self.__imuMsg.linear_acceleration_covariance = [ 0.04 , 0 , 0,
                                                         0 , 0.04, 0,
                                                         0 , 0 , 0.04 ]
        #rospy.init_node('arduino_monitor', anonymous=True)
        #rate = rospy.Rate(10) # 10hz

    def run(self):
        while self.__running:
            time.sleep(.02)
            if _arduino_serial_port.inWaiting():
                #print "theres something!"
                self.__sensorMsg = _arduino_serial_port.readline()
                #print self.__sensorMsg
                self.__sensorReadings = self.__sensorMsg.split(',')
                #print "I just split the msg"
                #print self.__sensorReadings

                #checksum check
                if not(self.data_is_valid(self.__sensorReadings)):
                    pass
                #valid message
                else:
                    #Publish IMU
                    if self.__sensorReadings[0] == 'i':
                        yaw =  float(self.__sensorReadings[1])
                        pitch = float(self.__sensorReadings[2])
                        roll = float(self.__sensorReadings[3])
                        self.__imuMsg.linear_acceleration.x = -float(self.__sensorReadings[4]) * self.__accel_factor
                        self.__imuMsg.linear_acceleration.y = float(self.__sensorReadings[5]) * self.__accel_factor
                        self.__imuMsg.linear_acceleration.z = float(self.__sensorReadings[6]) * self.__accel_factor
                        self.__imuMsg.angular_velocity.x = float(self.__sensorReadings[7])
                        self.__imuMsg.angular_velocity.y = -float(self.__sensorReadings[8])
                        self.__imuMsg.angular_velocity.z = -float(self.__sensorReadings[9])
                        q = quaternion_from_euler(roll,pitch,yaw)
                        self.__imuMsg.orientation.x = q[0]
                        self.__imuMsg.orientation.y = q[1]
                        self.__imuMsg.orientation.z = q[2]
                        self.__imuMsg.orientation.w = q[3]
                        self.__imuMsg.header.stamp = rospy.Time.now()
                        self.__imuMsg.header.seq = self.__seq
                        self.__seq += 1
                        self.__imu_pub.publish(self.__imuMsg)
                        self.__batteryLevel = int(self.__sensorReadings[10])
                        self.__battery_pub.publish(self.__batteryLevel)
                        #print "IMU"

                    #publish GPS
                    elif self.__sensorReadings[0] == 'g':
                        print "GPS"
                    #publish Ultra 1,4,6
                    elif self.__sensorReadings[0] == 'u':
                        order_u = [5,0,3]
                        print self.__sensorReadings
                        self.publish_ultrasonic(order_u)
                        #print "Ultra1"
                    #publish Ultra 2,5
                    elif self.__sensorReadings[0] == 'v':
                        order_v = [4,1]
                        self.publish_ultrasonic(order_v)
                        #print "Ultra2"
                    #publish Ultra 3,7
                    elif self.__sensorReadings[0] == 'w':
                        order_w = [6,2]
                        self.publish_ultrasonic(order_w)
                        #print "Ultra3"
                    else:
                        print "Invalid"
            else:
                pass

    #publishes ultrasonic data in the order supplied
    def publish_ultrasonic(self, order):
        for i in range(len(order)):
            self.__ultrasonic_msg.range = float(self.__sensorReadings[i+1])
            self.__ultrasonic_msg.header.frame_id = "learner_ultrasonic_" + str(order[i]) + "_link"
            self.__ultrasonic_msg.header.stamp= rospy.Time.now()
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
