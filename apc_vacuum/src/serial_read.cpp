/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, UT Arlington
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of UT Arlington nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#include "serial/serial.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string.h>
#include <sstream> 
#include <iostream>
#include <cstdio>
#include "apc_vacuum/ValveCtrl.h"

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif



using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;
using std::stringstream;


 // port, baudrate, timeout in milliseconds
 serial::Serial my_serial("/dev/ttyUSB0", 115200, serial::Timeout::simpleTimeout(2000));


// Wait for miliseconds
void my_sleep(unsigned long milliseconds) 
{
#ifdef _WIN32
      Sleep(milliseconds);
#else
      usleep(milliseconds*1000);
#endif
}



bool valve_ctrl_cmd(apc_vacuum::ValveCtrl::Request  &req, apc_vacuum::ValveCtrl::Response &res)
{
	//my_serial.readline();
	res.ack.clear();
	if(req.cmd == "ON")
	{
		my_serial.write("ON\n");

		my_serial.readline(res.ack);
	}
	else if(req.cmd == "OFF")
	{
		my_serial.write("OFF\n");

		my_serial.readline(res.ack);
	}
	req.cmd.clear();

	//ROS_INFO_STREAM("res.ack ="<<res.ack);
	return true;
}





int run(int argc, char **argv)
{
  string pressure_data;		// Contain the pressure data from arduino
  std_msgs::String sensor;   // Change the response data into integer to publish in a topic

  ros::init(argc, argv, "SerialCom");
  ros::NodeHandle n;
  ros::Publisher serial_pub = n.advertise<std_msgs::String>("serial", 2000);
  ros::Rate loop_rate(10);  // Loop will run at 10hz
  
  ros::ServiceServer service = n.advertiseService("valve_control", valve_ctrl_cmd);




  cout << "Is the serial port open?";
  if(my_serial.isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;



  while (ros::ok())
  {
    my_serial.write("DATA\n");
    my_serial.readline(pressure_data);

    
    sensor.data = pressure_data;

	serial_pub.publish(sensor);
	
	sensor.data.clear();
	pressure_data.clear();

    ros::spinOnce();
	loop_rate.sleep();
    
  }
  cout << "\nClosing Serial Port.\n";
  my_serial.close();


  return 0;
}







int main(int argc, char **argv) 
{
  do
  {
	try
	{
		run(argc, argv);
	}
	catch (exception &e)
	{
		cerr <<"Unhandled Exception: " << e.what() << endl;
		my_serial.close();
	}
	my_sleep(3000);
	my_serial.open();


  }while(1);
	
	return 0;
}
