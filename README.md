# Amazon Picking Challenge (APC)

This repository contains software developed by *Plocka Packa* for the 2015 Amazon Picking Challenge. The team members were part of the [Next Generation Microsystems and Robots](http://www.uta.edu/ee/ngs/ "NGS") Group headed by UT Arlington Electrical Engineering Professor [Dan Popa](https://www.linkedin.com/in/dan-popa-34361310 "https://www.linkedin.com/"):
* [Sven Cremer](http://www.svencremer.com/ "http://www.svencremer.com/")
* [Isura Ranatunga](http://isura.me/ "http://isura.me/")
* [Sumit Das](https://www.linkedin.com/profile/view?id=318963960 "https://www.linkedin.com/")
* [Fahad Mirza](https://www.linkedin.com/pub/fahad-mirza/29/aa2/98b "https://www.linkedin.com/")
* Rommel Alonzo
* Joshua Baptist

## Platform
The master branch is compatible with the PR2 robotic platform running ROS Groovy in Ubuntu 12.04. All packages use the *catkin* build system.

## ROS Packages
  * *apc_cortex*: contains state machine nodes
  * *apc_msgs*: defines custom ROS messages and services
  * *apc_robot*: classes with ROS API for moving the head, base, torso, grippers, and arms
  * *apc_json*: reads json files and makes the data available via ROS API
  * *apc_marker*: detects and publishes the pose of QR markers
  * *apc_object_detection*: interacts and handles data from Object Recognition Kitchen (ORK)
  * *apc_pcl*: manipulates and filters raw point clouds
  * *apc_shelf*: tools for detecting and localizing the shelf and individual bins
  * *apc_vacuum*: controls vacuum suction cups and reads pressure data via a serial communications channel
  * *apc_simulator*: launches Gazebo simulation environments for testing
  * *apc_workspace*: scripts and configuration files for setting up workspace
  * *libraries*: third-party software (mainly used by the *apc_vacuum* package)

# Install ROS
Install ROS Groovy on a Ubuntu 12.04 machine using the instructions available at

    http://wiki.ros.org/groovy/Installation/Ubuntu

Next, setup a workspace:  

    source /opt/ros/groovy/setup.bash
    mkdir ~/apc_ws/src -p
    cd ~/apc_ws/src
    catkin_init_workspace
    cd ~/apc_ws
    catkin_make

Open bashrc with an editor, i.e.

    gedit ~/.bashrc

and add the following to the end of the file:  

    export ROBOT=sim
    export ROS_MASTER_URI=http://localhost:11311
    source ~/apc_ws/devel/setup.bash

# Install Dependencies
The following needs to be installed (see drone.io file):

    sudo apt-get update
    sudo apt-get install python-rosinstall
    sudo apt-get install git-core python-argparse python-wstool python-vcstools python-rosdep
    sudo apt-get install ros-groovy-pr2-desktop
    sudo apt-get install ros-groovy-moveit-full
    sudo apt-get install ros-groovy-control-msgs
    sudo apt-get install ros-groovy-object-*
    sudo apt-get install ros-groovy-ar-track-alvar

In ROS groovy, *catkin* will not be able to find *move_base_msgs* since it is a rosbuild based package. Therefore, download the catkinized navigation stack in your workspace:

    cd ~/apc_ws/src
    git clone -b groovy-devel-catkinized https://github.com/ros-planning/navigation.git

Also download the catkinized *pr2_common_action_msgs* package:

    git clone -b hydro-devel https://github.com/PR2/pr2_common_actions.git

# Install APC
Download the APC repository in your workspace:

    cd ~/apc_ws/src
    git clone https://github.com/amazon-picking-challenge/plocka_packa.git
  
To compile the full APC package you still need to install PCL 1.7. However, to check if all other depenscies are met, temporary remove the *apc_pcl* folder and try to compile the workspace:

    cd ~/apc_ws
    catkin_make

To import the workspace into Eclipse IDE, do this once:

    cd ~/apc_ws
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles"

# Install ORK

-  Workspace Install

		mkdir obj_ws && cd obj_ws
 		wstool init src `rospack find apc_workspace`/conf/apc_object_recog.rosinstall
		cd src && wstool update -j8
		cd .. && rosdep install --from-paths src -i -y

		catkin_make

		echo "source ~/devel/setup.bash" >> ~/.bashrc
		source ~/.bashrc

-  Building database
  

- Object Recognition Setup

		roslaunch openni2_launch openni2.launch
		rosrun rviz rviz

     - Select /camera/driver from the drop-down menu and enable the depth_registration checkbox.

				rosrun rqt_reconfigure rqt_reconfigure

- Object Recognition Using Tabletop

     - Table Detection

				rosrun object_recognition_core detection -c `rospack find object_recognition_tabletop`/conf/detection.table.ros.ork

     - Object Detection

				rosrun object_recognition_core detection -c  `rospack find object_recognition_tabletop`/conf/detection.object.ros.ork


- Object Recognition Using Linemod

     - Training

				rosrun object_recognition_core training -c `rospack find object_recognition_linemod`/conf/training.ork

     - Detection

				rosrun object_recognition_core detection -c  `rospack find object_recognition_linemod`/conf/detection.ros.ork

# Install PCL 1.7 in ROS Groovy

These instructions are based on the answer by Neil Traft here http://answers.ros.org/question/62979/how-do-i-use-pcl-17-with-groovy/

A compiled version of this workspace/package is available at https://www.dropbox.com/s/d3m744rbm7u8rl3/pcl_ws.zip?dl=0

- Make PCL workspace

		cd ~
		mkdir -p pcl_ws/src
		cd pcl_ws/src
		git clone https://github.com/ros-perception/perception_pcl.git -b groovy-unstable-devel
		cd perception_pcl
		git checkout 528328f
		cd ..
		catkin_init_workspace
		cd ..
		catkin_make_isolated    # this step takes awhile

- Build Catkin package

		source ~/pcl_ws/devel_isolated/setup.bash
		cd ~/catkin_ws
		catkin_make
		source devel/setup.bash

- Make sure to include the workspaces in .bashrc in this order

		# ROS
		source /opt/ros/groovy/setup.bash
		source ~/pcl_ws/devel_isolated/setup.bash
		source ~/catkin_ws/devel/setup.bash

Make sure you do not build your workspace from any old terminals with the old workspaces. It is better to close all terminals and eclipse and just reopen a terminal.

- Fix for the Error: no such instruction: `vfmadd312sd 120(%rdi),%xmm0,%xmm1' issue.

	Install gcc 4.8: http://askubuntu.com/questions/61254/how-to-update-gcc-to-the-latest-versionin-this-case-4-7-in-ubuntu-10-04

		sudo add-apt-repository ppa:ubuntu-toolchain-r/test
		sudo apt-get update; sudo apt-get install gcc-4.8 g++-4.8

		sudo update-alternatives --remove-all gcc 
		sudo update-alternatives --remove-all g++

		sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-4.8 20
		sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-4.8 20

		sudo update-alternatives --config gcc
		sudo update-alternatives --config g++
	
	Verify gcc version with

		gcc --version
		

# Build Status

[![Build Status](https://drone.io/bitbucket.org/nextgensystems/apc/status.png)](https://drone.io/bitbucket.org/nextgensystems/apc/latest)
