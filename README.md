# Titan AlphaPilot Racer

## Quad_Racer

This package named ```quad_racer``` is a C++ ROS node which generates ```rateThrust``` messages to control quadcopters. 

### Rostopics Published

```/uav/input/rateThrust```

### Requirements

This is a ROS package and expects ROS to be installed on the host machine running the ```quad_racer``` node. 

### Setup

Follow the steps provided below to download, build and launch the source code for ```quad_racer```

```
cd ~/catkin_ws/src
git clone https://github.com/alphapilotaichallenge/quad_racer.git
cd ..
catkin build
source devel/setup.bash
roslaunch quad_racer quad_racer_flightgoggles.launch ignore_collisions:=1 use_external_renderer:=1
```

### Steps to launch quad_racer after a Ubuntu reboot

```
rosrun flightgoggles FlightGoggles.x86_64 
cd catkin_ws/
cd src/quad_racer/
git pull
cd ../..
catkin build
source devel/setup.bash
roslaunch quad_racer quad_racer_flightgoggles.launch ignore_collisions:=1 use_external_renderer:=1
```

### To launch final challenge race course
```roslaunch quad_racer scorer.launch level:=final gate_locations:=1```

### To upload the latest changes (Remember to always pull the latest changes before executing these instructions)

```
git add -A
git commit -m "<your-comment-on-the-commit>"
git push origin master
```

### To download latest changes to flightgoggles and recompile
```
cd ~/catkin_ws/
cd src/flightgoggles
git pull origin master
cd ../
wstool update
cd ../
catkin clean 
catkin build
```

### Author
Bhavyansh Mishra




