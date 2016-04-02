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
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray

serial_port = sys.argv[1]

_arduino_serial_port = serial.Serial(serial_port,115200)
_arduino_serial_port.timeout = None    

def generate_checksum(data):
    sumOfBytes = 0
    for strings in data:
        for char in strings:
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
        self.__msg += str(generate_checksum(self.__msg.split(","))) 
        self.__msg += "\n"
	print self.__msg
    	#if not(self.__auto):
	#    if self.__msg != self.__old_msg:
	if time.clock() - self.__old_time >= 0.1: 
            _arduino_serial_port.write(self.__msg)
	    _arduino_serial_port.flush()
	    self.__old_time = time.clock() 
	self.__old_msg = self.__msg
        self.__msg = self.__msg[0]

#thread that handles incoming messages from the arduino
class ArduinoMonitor (threading.Thread):
    def __init__(self, threadID, name):
        threading.Thread.__init__(self)
        self.__threadID = threadID
        self.__name = name
        self.__accel_factor = 9.806 / 256.0
        self.__seq = 0
        self.__sensorMsg = ""
        self.__sensorReadings = []
        self.__ultrasonicData = zeros(7)
        self.__imuMsg = Imu()#Yaw,Pitch,Roll,Accel x,y,z,gyro x,y,z
        self.__batteryLevel = 0
        self.__running = 1
        self.__battery_pub = rospy.Publisher('arduino_sensors/battery_level', Int16,queue_size = 10)
        self.__ultrasonic_pub = rospy.Publisher('arduino_sensors/ultrasonic_array',Int16MultiArray,queue_size = 10)        
        self.__imu_pub = rospy.Publisher('arduino_sensors/imu',Imu,queue_size = 10)
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
            #time.sleep(.1)
            if _arduino_serial_port.inWaiting():
		print "Just received"
                self.__sensorMsg = _arduino_serial_port.readline()
                print self.__sensorMsg
		
                self.__sensorReadings = self.__sensorMsg.split(',')            
		#print "I just split the msg"
                #print self.__sensorReadings
                
                if not(self.data_is_valid(self.__sensorReadings)):
                    pass
                else:
                    pass
                    
    #validates message using the checksum        
    def data_is_valid(self, data):
        checksum = int(data.pop())
        if generate_checksum(data)  == checksum:
            #print "valid data"
            return True
        else:
            #print "invalid data"
	    #print checksum
	    #print generate_checksum(data)
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
'''
def signal_handler(signal, frame):
    print('You pressed Ctrl+C!')
'''
sys.exit(0)
_arduino_serial_port.close()
