import pygame
import serial
import time

ser = serial.Serial('COM11', 57600)
ser.timeout = None
ser.write("j")
time.sleep(3)
print "Ready"

pygame.init()
pygame.joystick.init()

done = False
joystick_count = pygame.joystick.get_count()
print "There are  %d  joystick(s) connected" % (joystick_count)
joystick = pygame.joystick.Joystick(0)
joystick.init()
name = joystick.get_name()
print name
axes = joystick.get_numaxes()
buttons = joystick.get_numbuttons()

lastCommand = ""

while done==False:
    #joystick = pygame.joystick.Joystick(0)
    #joystick.init()
 
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
        print command
        lastCommand = command
        
    #button 12 for quit
    #ser.flushOutput()
    if joystick.get_button(12):
        ser.close()
        print "Now exiting"
        done = True

    #print command
    print ser.readline()
    time.sleep(.01)
pygame.quit()
