import time
import rospy
from sensor_msgs.msg import Joy
from std_msgs.msg import Int32

class PS3Controller:
    def __init__(self):
        "PS3 controller node constructor"
        # tele-operation controls
        self._steer = 0                  # steering axis
        self._drive = 13                  # shift to Drive
        self._reverse = 12                # shift to Reverse
        self._brake = 13
        self._run_suspend = 6
        self._stop = 11
        self._auto = False
        self._disable = False
        self._command_publisher = rospy.Publisher('arduino_sensors/command', Int32,queue_size = 1)
        # initialize ROS joystick suscriber and steering and throttle publisher
        self._joy = rospy.Subscriber('joy', Joy, self.joy_callback)
        
    def joy_callback(self, joy):
        "invoked every time a joystick message arrives" 
        
        #TODO debounce the button presses
        if joy.buttons[self._run_suspend]:
            self._auto = not(self._auto)
            if self._auto:
                rospy.logwarn('autonomous mode enabled')
            else:
                rospy.logwarn('teleop mode enabled')
        if joy.buttons[self._stop]:
            self._disable = not(self._disable)
            if self._disable:
                rospy.logwarn('\nVEHICLE STOPPED\n')
            else:
                rospy.logwarn('vehicle released')

        if not(self._auto) and not(self._disable):
            # create the throttle and steering command
            if joy.buttons[self._brake]:
                self._command = 1
            else:
                self._command = 0
            self._command += int(-30 * joy.axes[self._steer]) << 1
            self._command += int(255 *( -joy.axes[self._drive] + joy.axes[self._reverse])) << 8;
            #print "0x%x" % int(self._command)
            self._command_publisher.publish(self._command)
        ''''
        freq = 20.0
        #reduce the controller frequency to 10Hz and write to the arduino
        if time.clock() - self._old_time >= (1.0/freq):
            _arduino_serial_port.write(self._msg)
            #_arduino_serial_port.flush()
            self._old_time = time.clock()
            #print self._msg
        self._old_msg = self._msg
        self._msg = "r,"    #reset message
        '''''

