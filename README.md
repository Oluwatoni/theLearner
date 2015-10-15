# theLearner
The learner is an Arduino powered autonomous RC car that can avoid obstacles and send sensor data,via Bluetooth, to a computer to visualize the data recieved. The Learner can be controlled with a PS4 controller when not in autonomous mode. It has a four ultrasonic sensors, a GPS and a 9 degree of freedom IMU. 
The electrical distribution board was designed in EagleCAD and soldered by hand. The sensors utilize UART and SPI protocol to communicate and have been calibrated for this specific purpose. 
The visualization and joystick controller interface was written in Python using Pygame, Vpython, numPy, WxPython and PySerial
