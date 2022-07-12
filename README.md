# Four-legged Robot coding tutorial

This is a commercial project which required for creating a coding tutorial to teach students to use Arduino to control a robot dog called bittle (https://www.petoi.com/pages/bittle-open-source-bionic-robot-dog). 

I was provided with several of these robots and required to design a several tasks for learners to accomplish. Code for all the tasks can be found in the repository. 

Bittle is a four-legged robot, so the first thing need to be done is to design the gaits for it. The robot contains 8 motors. The angles   for motors in each frame is stored in an array. Commands for each motor will be repeated according to the value in the array. In this way, the robot could perform some gaits like walking and running. An infrared remote controller is also provided to enable communication between the robot and the controller.

The robot has an embedded IMU chip in it, so I designed a self balancing task for the robot to remain stable on a shaking platform. This task utilized the PID algorithm, each motor's angle is changed based on the output of PID algorithm to compensate for the tilt of platform that it stands on. The body of the robot can remain horizonal while the platform is keep shaking. 

In addition, peripheral sensors like ultrasonic ranging unit and camera are used to enable interaction with the environment. With the help of these sensors, the robot can avoid obstacles or following behind the user.

<img src="D:./1.jpg" alt="1" width="600" />

