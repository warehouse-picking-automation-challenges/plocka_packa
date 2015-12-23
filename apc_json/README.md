ROS package with JSON reader. There are tools for reading workorders as well as grasping strategies.
### Parser
Loads the mapping between the object names and IDs into the param server:

    roslaunch apc_json apc_parser.launch

Provides a user interface for loading JSON workorder:

    rosrun apc_json apc_parser

Once ready, it generates the ORK files used for object detection and provides rosservices for accessing relevant data:

* /apc/json/bin_contents
* /apc/json/work_order
* /apc/json/status

### Grasping database   
Generates the configuration files containing the grasping strategies of each object:

    rosrun apc_json grasping_json_generator.py

Starts the grasping node and loads the strategies files from /config/grasping:

    roslaunch apc_json grasping.launch

The startegies can be accessed via the rosservice

* /apc/grasp_strategy


