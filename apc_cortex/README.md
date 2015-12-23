High level planner for the APC. Motion clients will be included as objects. All other functionalities (object detecton, JSON parser...) will be implemented in separate nodes and data will be transfered via ROS messages.


Testing
-------
Running state machine
```
roslaunch apc_cortex apc_cortex.launch
rosrun apc_cortex apc_cortex_node
```

Note, make sure that the odom_origin is initialized corectly after running apc_cortex.launch. For example,
```
rosservice call /apc/tf/set_odom_origin "pose:
  position:
    x: 1.4
    y: 0.5
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" 
```


Procedure for adding new grasping strategies
-------

- To add a new grasping strategy, 2 files must be modified in 3 different places

 	- apc_cortex/include/apc_cortex.h: Add your grasping strategy's name to the enumaerated type `GraspStrategy` in the ApcCortex class
	- apc_cortex/include/apc_cortex.cpp:
		- updateGraspingStrategy(): add to the if-else statement
		- grasp(): add to the switch statement and add the procedure to the case
