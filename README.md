# TheLearner
The learner is a mobile robotics platform built to learn mapping, localization and path planning. The learner sports seven ultrasonic sensors, an encoder and a 9 DOF IMU.

It is currently capable of mapping with known poses. The next goal is indoor exploration while SLAMing.

Todo:

- move the embedded code to the periodic scheduler
- switch imus
- swith the steering servo and design a better steering mechanism
- complete the sonar->lidar transition for mapping
- 3d print new enclosures
- buy a neat container for transporting the robot
- local obstacle avoidance on the robot(VFH+?)
  - rough pose estimation on the robot
- turn off motion commands if nothing was recieved recently(define recently)
- use scan matching to relocalize or detect and track the position of features in the map?(EKF-SLAM)
- implement an exploration algorithm (Frontier based? taking into account the amount of information that could be obtained from each frontier)
  - This can be contain a rosaction for navigation to the goal postition
  - implement a globalish path planner for frontier navigation
- move code to a raspberry pi?
- build or find a simulation of this robot for some of the higher level algorithm testing(MATLAB vs Gazebo)?
