from visual import *
from numpy import array
#import time
import serial

MyScene=display(title='My Virtual World') #Create your scene and give it a title.
 
MyScene.width=800  #We can set the dimension of your visual box. 800X800 pixels works well on my screen
MyScene.height= 800
MyScene.autoscale=False #We want to set the range of the scene manually for better control. Turn autoscale off
MyScene.range = (12,12,12) #Set range of your scene to be 12 inches by 12 inches by 12 inches. 
target=box(length=.1, width=10,height=5, pos=(-6,0,0)) #Create the object that will represent your target
myBoxEnd=box(length=.1, width=10,height=5, pos=(0,0,0))


ser = serial.Serial('COM16', 57600)
ser.timeout = None
ser.write("start!")
#time.sleep(3)
print "Ready"

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False

def read_from_arduino_port(command):
    ser.write(command)
    return ser.readline()

sensor_readings = zeros((11))
def valid_message(arduino_message):
    dataNums=arduino_message.split(',')
    if len(dataNums) < 10:
        print str(dataNums) + "too small"
        return False
    index = 0
    for dataNum in dataNums:
        if not(is_number(dataNum)):
            print str(dataNum) + "not a real number"
            return False
        sensor_readings[index] = float(dataNum)
        index+=1
    return True


while True:
    #time.sleep(100)
    rate(25)
    while(ser.inWaiting()==0): # Wait here untill there is data on the Serial Port
        pass                          # Do nothing, just loop until data arrives
    #ser.flushInput()
    textline = ser.readline()     # read the entire line of text
    if not(valid_message(textline)):
        continue
    dataNums=textline.split(',')       #Remember to split the line of text into an array at the commas
    front=sensor_readings[0]             # Make variables for Red, Blue, Green. Remember
    left=sensor_readings[1]            # the array was read as text, so must be converted
    right=sensor_readings[2]           # to numbers with float command
    back=sensor_readings[3]            # last number in the list is the distance
    print sensor_readings

ser.close()

