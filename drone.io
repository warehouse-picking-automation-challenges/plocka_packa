# Install ROS Groovy in Ubuntu 12.04

    sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu precise main" > /etc/apt/sources.list.d/ros-latest.list'
    wget http://packages.ros.org/ros.key -O - | sudo apt-key add -
    sudo apt-get update
    sudo apt-get install ros-groovy-desktop-full

    sudo rosdep init
    rosdep update

    echo "source /opt/ros/groovy/setup.bash" >> ~/.bashrc
    source ~/.bashrc

# Install dependencies

    sudo apt-get update
    sudo apt-get install python-rosinstall
    sudo apt-get install git-core python-argparse python-wstool python-vcstools python-rosdep
    sudo apt-get install ros-groovy-moveit-full
    sudo apt-get install ros-groovy-control-msgs
    sudo apt-get install ros-groovy-object-*
    sudo apt-get install ros-groovy-ar-track-alvar
    

# Setup Catkin Workspace

    source /opt/ros/groovy/setup.bash
    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws
    catkin_make

    echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

# Setup MoveIt package

    cd ~/catkin_ws/src
    git clone https://github.com/ros-planning/moveit_pr2.git
    cd moveit_pr2
    git checkout groovy-devel

# Setup APC package

     cp -a /home/ubuntu/src/bitbucket.org/nextgensystems/apc ~/catkin_ws/src
     cd ~/catkin_ws
     catkin_make

# Setup APC Object Detection (optional)

    cd ~
    mkdir obj_ws && cd obj_ws
    wstool init src /home/ubuntu/src/bitbucket.org/nextgensystems/apc/apc_workspace/conf/apc_object_recog.rosinstall
    cd src && wstool update -j8
    cd .. && rosdep install --from-paths src -i -y

#    catkin_make

#    echo "source ~/devel/setup.bash" >> ~/.bashrc
#    source ~/.bashrc

