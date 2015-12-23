High level planner for the APC pick and place demo. Motion clients will be included as objects. All other functionalities (object detecton, JSON parser...) will be implemented in separate nodes and data will be transfered via ROS messages.

Setup
-------
Basestation:
```
roslaunch apc_cortex pick_and_place.launch
roslaunch pr2_navigation_global rviz_move_base.launch
```
PR2 robot:
```
roslaunch pr2_2dnav pr2_2dnav.launch
rosrun map_server map_server /home/pr2admin/groovy_ws_cody/Git/utari-test/WaypointNav_pkg/maps/2nd_floor_newest.yaml
roslaunch apc_robot pr2_kinect.launch
roslaunch ar_track_alvar pr2_indiv_no_kinect.launch
```
First set the current location of the PR2. As the PR2 begins to move, amcl package will begin to converge to a more precise location. 

Running
-------
```
rosrun apc_cortex cortex_pnp_node
```
Current Testing
-------
- [ ] Base movement
   - [x] Navigation Stack (apc_robot/Base)
   - [ ] Command Velcity (apc_robot/BaseMovement)
