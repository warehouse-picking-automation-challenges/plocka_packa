
# README #


######  V2.0  ######

Handled 'SerialException', everytime the code throws exception its gonna open the port again.
The only catch is you cannot close the node with 'CTRL+C'. Instead do, 'CTRL+Z'
and then type "'kill %1' (Enter)", twice.







######  V1.0  ######

### What is this repository for? ###

* APC from UT at Arlington. Team: Plocka Packa
* Version 1.0


### How do I get set up? ###

1) To have root access of the tty ports add your user name as in the user group:

sudo usermod -a -G dialout $USER

Then logout and login again

2) Make sure you have these two libraries in catkin_ws/src/:

git clone https://github.com/ros/cmake_modules.git

git clone https://github.com/wjwwood/serial.git


Run cmd: rosrun apc_vacuum serial_read


topic name --> /serial

service name --> /valve_control

Service cmd --> ON / OFF (string) 
ACk --> ON/OFF done


### Who do I talk to? ###

* Fahad Mirza
* fahad.mirza34@mavs.uta.edu
