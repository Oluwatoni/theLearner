#from visual import *
from __future__ import division, print_function
import time
import serial
from visual import *
import visual as vs   # for 3D panel
import wx
from numpy import array
import math
import scipy

#interface
win = vs.window(width=1920, height=1080, menus=False, title='Learner display')
                         # make a main window. Also sets w.panel to addr of wx window object.

scene = vs.display(window = win,
     x=0, y=0, width=1700, height=1000,
     center=(0,0,0), background=(0,0,0),forward=-vs.vector(0,0,1))

userzoom = False
right = 10
left = 20
up = 15
down = 13

#TODO account for ultrasonic offset
#TODO add wheels to 3D model
robot = frame()
mybox = box(frame = robot, pos=(0,0,0), size=(14,21,10),color= color.red)
upWall = box(frame = robot, pos=((right - left)/2,up,0), size=((left + right),0.5,4),color= color.cyan)
downWall = box(frame = robot, pos=((right - left)/2,(-1 * down),0), size=((left + right),.5,4),color= color.cyan)
leftWall = box(frame = robot, pos=((-1 * left),(up - down)/2,0), size=(.5,(up + down),4),color= color.cyan)
rightWall = box(frame = robot, pos=(right,(up - down)/2,0), size=(.5,(up + down),4),color= color.cyan)
pointer = arrow(frame = robot, pos=(0,-2,10), axis=(0,4,0), shaftwidth=3,headlength =10,headwidth= 6)
currentAngle = 0
win.fullscreen = True

x1 = scene.width + 5
y1 = 0
pan = win.panel   # addr of wx window object
pan.SetSize( (1920,1080)) # work-around to make display work.  >= size of client area of window.

heading = 0
speed = 0
gps = [0,0]

#text(text="Heading: " + str(heading) +"\nspeed: " + str(speed) + "\nGPS: " + str(gps) ,pos=(scene.width/100,scene.height/100,0),
    #align='right', depth=-0.3, color=color.white)
    
# Controls (= widgets) have to be put in the wx window.
# Positions and sizes are in pixels, and pos(0,0) is the upper left corner of the window.

wx.StaticText( pan, pos=(x1,y1),
    label = "Welcome to the\nlearner robot Visualizer")

#communication

ser = serial.Serial('COM4', 57600)
ser.timeout = None
ser.write("start!")
time.sleep(3)
print ("Ready")

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

def read_from_arduino_port(command):
    ser.write(command)
    return ser.readline()

maxReading = 12
sensor_readings = zeros((maxReading))
def valid_message(arduino_message):
    dataNums=arduino_message.split(',')
    if len(dataNums) < 10 or len(dataNums) > maxReading:
        print (str(dataNums) + "irregular size")
        return False
    index = 0
    for dataNum in dataNums:
        if not(is_number(dataNum)):
            print (str(dataNum) + "not a real number")
            return False
        sensor_readings[index] = float(dataNum)
        index+=1
    return True

while True:
    sleep(10)
    #visual.rate(500)
    while(ser.inWaiting()==0): # Wait here untill there is data on the Serial Port
        pass                          # Do nothing, just loop until data arrives
    #ser.flushInput()
    textline = ser.readline()     # read the entire line of text
    if not(valid_message(textline)):
        continue
    dataNums=textline.split(',')       #Remember to split the line of text into an array at the commas
    up=sensor_readings[0]             # Make variables for Red, Blue, Green. Remember
    left=sensor_readings[1]            # the array was read as text, so must be converted
    right=sensor_readings[2]           # to numbers with float command
    down=sensor_readings[3]            # last number in the list is the distance
    print (sensor_readings)
    
    #update visuals
    upWall.pos = vector((right - left)/2,up,0)
    upWall.size =  ((left + right),.5,4)
    leftWall.pos = vector ((-1 * left),(up - down)/2,0)
    leftWall.size =  (.5,(up + down),4)
    rightWall.pos = vector (right,(up - down)/2,0)
    rightWall.size =  (.5,(up + down),4)
    downWall.pos = vector ((right - left)/2,(-1 * down),0)
    downWall.size = ((left + right),.5,4)

    #update orientation
    yaw = sensor_readings[5]
    robot.rotate(angle= (yaw-currentAngle), axis =(0,0,1), origin=(0,0,0))
    currentAngle = yaw

    #update position
    accX = sensor_readings[9]
    accY = sensor_readings[10]
    acc = math.sqrt(accX**2 + accY**2)
    timeSinceLastPoll = sensor_readings[6]
    displacement = scipy.integrate.cumtrapz(scipy.integrate.cumtrapz(acc, dx=timeSinceLastPoll), dx=timeSinceLastPoll)
    mybox.pos = vector(math.cos(yaw)*displacement
                       + mybox.x,math.sin(yaw)*displacement + mybox.y,0)
    
ser.close()



