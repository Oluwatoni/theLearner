from __future__ import division, print_function
from visual import *
import visual as vs   # for 3D panel 
import wx  
import numpy

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

mybox = box(pos=(0,0,0), size=(4,4,4),color= color.red)
upWall = box(pos=((right - left)/2,up,0), size=((left + right),0.5,4),color= color.cyan)
downWall = box(pos=((right - left)/2,(-1 * down),0), size=((left + right),.5,4),color= color.cyan)
leftWall = box(pos=((-1 * left),(up - down)/2,0), size=(.5,(up + down),4),color= color.cyan)
rightWall = box(pos=(right,(up - down)/2,0), size=(.5,(up + down),4),color= color.cyan)
pointer = arrow(pos=(0,-2,4), axis=(0,4,0), shaftwidth=1,headlength =2,headwidth= 2)

win.fullscreen = True

x1 = scene.width + 5
y1 = 0
pan = win.panel   # addr of wx window object 
pan.SetSize( (1920,1080)) # work-around to make display work.  >= size of client area of window.  

heading = 0
speed = 0
gps = [0,0]
text(text="Heading: " + str(heading) +"\nspeed: " + str(speed) + "\nGPS: " + str(gps) ,pos=(scene.width/100,scene.height/100,0),
    align='right', depth=-0.3, color=color.white)
# Controls (= widgets) have to be put in the wx window. 
# Positions and sizes are in pixels, and pos(0,0) is the upper left corner of the window.

wx.StaticText( pan, pos=(x1,y1),
    label = "Welcome to the\nlearner robot Visualizer")

for num in range(0,100):
    rate(10)
    rightWall.pos = vector (num,(up - down)/2,0)
