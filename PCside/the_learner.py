from __future__ import division, print_function
import pygame
import serial
import time
import serial
from visual import *
import visual as vs   # for 3D panel
import wx
from numpy import array
import math
#import scipy

#TODO create separate threads for the visualizer and control
#interface
win = vs.window(width=1920, height=1080, menus=False,
                title='Learner display')
                
scene = vs.display(window = win,
     x=0, y=0, width=1700, height=1000,
     center=(0,0,0), background=(0,0,0),forward=-vs.vector(0,0,1))

userzoom = False
right = 10
left = 20
up = 15
down = 13
centerOfG = vector(0,0,0)

#TODO account for ultrasonic offset
#TODO add wheels to 3D model
robot = frame(make_trail=True)
mybox = box(frame = robot, pos=centerOfG, size=(14,21,10),
            color= color.red)
upWall = box(frame = robot, pos=((right - left)/2,up,0),
             size=((left + right),0.5,4),color= color.cyan)
downWall = box(frame = robot, pos=((right - left)/2,(-1 * down),0),
               size=((left + right),.5,4),color= color.cyan)
leftWall = box(frame = robot, pos=((-1 * left),(up - down)/2,0),
               size=(.5,(up + down),4),color= color.cyan)
rightWall = box(frame = robot, pos=(right,(up - down)/2,0),
                size=(.5,(up + down),4),color= color.cyan)
pointer = arrow(frame = robot, pos=(0,-2,10), axis=(0,4,0),
                shaftwidth=3,headlength =10,headwidth= 6)
currentAngle = 0
win.fullscreen = True

x1 = scene.width + 5
y1 = 0
pan = win.panel   # addr of wx window object
pan.SetSize( (1920,1080)) 
heading = 0
speed = 0
gps = [0,0]
    
wx.StaticText( pan, pos=(x1,y1),
    label = "Welcome to the\nlearner robot Visualizer")

#communication
ser = serial.Serial('COM11', 57600)
ser.timeout = None
ser.write("start!")
time.sleep(3)
print ("Ready")

pygame.init()
pygame.joystick.init()

done = False
joystick_count = pygame.joystick.get_count()
print ("There are  %d  joystick(s) connected" % (joystick_count))
joystick = pygame.joystick.Joystick(0)
joystick.init()
name = joystick.get_name()
print (name)
axes = joystick.get_numaxes()
buttons = joystick.get_numbuttons()

lastCommand = ""

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
    if len(dataNums) != maxReading:
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

def integrateTrapezoid(a,b,h):
    return 0.5*(a+b)*float(h)

oldVelocity = 0
oldVelocityX = 0
oldVelocityY = 0
while done==False:
    sleep(0.01)
    #visual.rate(500)
    while(ser.inWaiting()==0): # Wait here until there is data on the Serial Port
        pass                              
    textline = ser.readline()     # read the entire line of text
    if not(valid_message(textline)):
        continue
    dataNums=textline.split(',')       #split the line of text into an array at the commas
    up=sensor_readings[0]             
    left=sensor_readings[1]            
    right=sensor_readings[2]           
    down=sensor_readings[3]            
    print (sensor_readings)
    
    #update ultrasonic sensor readings
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
    newAccX = (sensor_readings[9] / 255.0) * 980
    newAccY = (sensor_readings[10] / 255.0) * 980
    newAcc = math.sqrt(newAccX**2 + newAccY**2)
    oldAccX = (sensor_readings[7] / 255.0) * 980
    oldAccY = (sensor_readings[8] / 255.0) * 980
    oldAcc = math.sqrt(oldAccX**2 + oldAccY**2)

    timeSinceLastPoll = sensor_readings[6]/1000.0
    print ("timeSinceLastPoll " +str(timeSinceLastPoll))

    newVelocityX = integrateTrapezoid(oldAccX,newAccX,timeSinceLastPoll)
    newVelocityY = integrateTrapezoid(oldAccY,newAccY,timeSinceLastPoll)
    displacementX = integrateTrapezoid(oldVelocityX,newVelocityX,
                                       timeSinceLastPoll)
    displacementY = integrateTrapezoid(oldVelocityY,newVelocityY,
                                       timeSinceLastPoll)
    robot.pos = vector((math.sin(yaw)*displacementX)+mybox.x
                       +(math.sin(yaw+(math.pi/2))*displacementX),
                       (math.cos(yaw)*displacementY)+mybox.y
                       +(math.cos(yaw+(math.pi/2))*displacementY),0)    
    
    '''
    newVelocity = integrateTrapezoid(oldAcc,newAcc,timeSinceLastPoll)
    displacement = integrateTrapezoid(oldVelocity,newVelocity,timeSinceLastPoll)
    oldVelocity = newVelocity
    mybox.pos = vector(math.cos(yaw)*displacement
                       + mybox.x,math.sin(yaw)*displacement + mybox.y,0)
    '''

    #pygame control
    count = pygame.event.get()
                   
    command = "c"
    #axis 0 for left-right
    #axis 4 for forward
    #axis 5 for reverse

    
    #print joystick.get_axis(0)
    #print joystick.get_axis(1)
    #print joystick.get_axis(5)
    command += str(int(joystick.get_axis(0) * 10.4))
    #print command
    command += ","

    
    if joystick.get_axis(4) > joystick.get_axis(5):
        command += str(int((joystick.get_axis(4) +1) * 50.5 ))
        #print command
    else :
        command += str(-1 * int((joystick.get_axis(5) +1) * 50.5))
        #print command

    command += ","
    
    #button 5 for e-stop
    if joystick.get_button(5):
        command += "1"
    else:
        command += "0"
    
    command += "\n"
    #send command if it is different from the last command
    if (command != lastCommand):
        ser.write(command)
        print (command)
        lastCommand = command
        
    #button 12 for quit
    #ser.flushOutput()
    if joystick.get_button(12):
        ser.close()
        print ("Now exiting")
        done = True
pygame.quit()
