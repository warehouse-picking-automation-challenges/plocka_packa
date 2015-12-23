## Installation


To add the package models to Gazebo:
```
rosrun apc_simulator install_models.sh 
```


## Running

To start the simulation with the PR2 robot:
```
roslaunch apc_simulator apc_simulator_pr2.launch
```
To start the simulation with Baxter:
```
./baxter.sh sim
roslaunch apc_simulator apc_simulator_baxter.launch
```
To enable the robot
```
./baxter.sh sim
rosrun baxter_tools enable_robot.py -e
rosrun baxter_examples joint_position_keyboard.py
```
