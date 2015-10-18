#from visual import *
from numpy import *
import time
import serial

ser = serial.Serial('COM3', 57600)
ser.timeout = 0
ser.write("start!")
time.sleep(4)
print "Ready"

rec = "ci"
inte = ""

def is_number(s):
    try:
        int(s)
        return True
    except ValueError:
        return False

def read_from_arduino_port(command):
    ser.write(command)
    return ser.readline()

def parse_message(arduino_message):
    sensor_readings = zeros((11))
    indi = ""
    index = 0
    for char in arduino_message:
        if char == ",":
            try:
                sensor_readings[index] = float(indi)
            except ValueError:
                print indi                
            index+=1
            indi = ""
        elif char == "\n":
            sensor_readings[index] = float(indi)
        else: 
            indi += char
            #print indi
    print sensor_readings
    return sensor_readings

def valid_message(arduino_message):
    for digit in arduino_message:
        if not(digit == "," or digit == "." or digit == " " or not(is_number(digit)) or digit != "\n"):
            #print "restarting 2"
            return False
    return True

summ = 0
big = 30
for num in range(0,big):
    start = time.clock()
    end = time.clock()
    count = 0
    garbage_in_message = True
    
    while(end - start < 1):
        count+=1
        while garbage_in_message:
            #print "first"
            command = 'p'
            rec = read_from_arduino_port(command)
            if len(rec) < 9 or not(valid_message(rec)):
                #print "restarting 1"
                garbage_in_message = True
                continue
            garbage_in_message = False
            
        #print rec
        sensor_readings = parse_message(rec).view()
        end = time.clock()
        
        #print "count " , count
    summ+= count

print str(summ/big) + " is the average number of messages successfully transmitted every second"
print 'completed'
ser.close()

