ROS package for handling object detection.

### Yaml File

The Yaml fiel contains the name of all teh objects in the Couch DB database along with the unique key assigned to them.


Load the Yaml file from

*    apc_object_detection/config/object_ids.yaml


### Object Detection Node   

The object detection node listens to "recognized_object_array" topic and stores the list of objects identified along with their locations.


Run linemod to identify data and pubilsh to topic "recognized_object_array"


	rosrun object_recognition_core detection -c  'rospack find object_recognition_linemod'/conf/detection.ros.ork


Run object detection node to store object information


	rosrun apc_object_detection apc_object_detection_node

