## apc_baxter


This package contains a *BaxterCommander* class for enabling the robot. It also utilizes a *Gripper* class for controlling the grippers and a *BaxterMoveIt* class for moving, kinematics, and planning. 


## Running

Start the apc simulatior and then launch MoveIt:
```
rosrun baxter_interface joint_trajectory_action_server.py
roslaunch baxter_moveit_config demo_baxter.launch
```
To demo the work in progress:
```
rosrun apc_baxter testing_node /joint_states:=/robot/joint_states
```
or
```
roslaunch apc_baxter testing_node.launch
```
Tutorials/Code for testing:
```
roslaunch apc_baxter_moveit kinematic_model
rosrun apc_baxter_moveit movegroup_test
```
### Video demo
```
roslaunch apc_baxter apc_baxter.launch
rosrun apc_baxter test_video_demo /joint_states:=/robot/joint_states
```