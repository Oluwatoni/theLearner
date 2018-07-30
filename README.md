# Imọ
Imọ is a mobile robotics platform built to learn mapping, localization and path planning. The learner sports seven ultrasonic sensors, an encoder and a 9 DOF IMU.

It is currently capable of mapping. The next goal is indoor exploration while SLAMing.

Todo(in no particular order):

- remove the side and rear ultrasonic sensors
- move the embedded code to the periodic scheduler
- change the commands to a steering angle and a linear velocity magnitude
- add a position server to the kalman filter node? or just get the tf between the points of interest
- combine the laser reads into a full 360 laser scan message
- rewrite the mapping node(to cpp)
- rewrite the EKF node(to cpp)
- characterize the lidar and tune a closed loop controller
- characterize the velocity and tune a closed loop controller
- add motor easing to the mix?
- complete the sonar->lidar transition for mapping
- buy a neat container for transporting the robot
- local obstacle avoidance on the robot(VFH+?)
  - rough pose estimation on the robot
- turn off motion commands if nothing was recieved recently(define recently)
- use scan matching to relocalize(hector or cartographer)or detect and track the position of features in the map(EKF SLAM or FastSLAM)?
- implement an exploration algorithm (Frontier based? taking into account the amount of information that could be obtained from each frontier)
  - This can be contain a rosaction for navigation to the goal postition
  - implement a globalish path planner for frontier navigation
- move code to a raspberry pi?
- build or find a simulation of this robot for some of the higher level algorithm testing(MATLAB vs Gazebo)?
