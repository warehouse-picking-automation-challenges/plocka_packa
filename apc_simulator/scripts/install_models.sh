#!/bin/bash         
#
# Script to permanently add package model folder to the Gazebo model path
#
# Sven Cremer
# 27 Nov 2014
#
# Use: ./install_models.sh
#

echo 'export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(rospack find apc_simulator)/models' >> ~/.bashrc
echo "Added $(rospack find apc_simulator)/models to GAZEBO_MODEL_PATH"
echo "Please source ~/.bashrc"
