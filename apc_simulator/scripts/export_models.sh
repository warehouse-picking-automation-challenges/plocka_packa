#!/bin/bash         
#
# Script to add models folders inside package to the Gazebo model path
#
# Sven Cremer
# 27 Nov 2014
#
# Use: ./export_models.sh [rospkg]
#
 
function showHelp(){
    echo
    echo "Script to add models inside package to the Gazebo model path."
    echo "Place it in the 'script' folder of your catkin package"
    echo "and make sure that the file is executable"
    echo "and that the models are inside rospkg/models."
    echo
    echo "Use: ./export_models.sh [rospkg]"
    echo "Or from a roslaunch file:"
    echo
    echo '<launch>'
    echo '  <node pkg="rospkg_with_script_folder" type="export_models.sh"'
    echo '    args="rospkg_name_with_models"'
    echo '    name="export_models" output="screen">'
    echo '  </node>'
    echo '</launch>'
}

if [ "$1" = "-h" ]||[ -z "$1" ]; then
    showHelp
else
    echo "export GAZEBO_MODEL_PATH=$(rospack find $1)/models"
    export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find $1)/models
fi
