// keyboard_main.cpp
// Jake Ware and Jarvis Schultz
// Spring 2011


//---------------------------------------------------------------------------
// NOTES
//---------------------------------------------------------------------------

/*
int operating_condition = [0,3]
0: Idle
1: Calibrate
2: Run
3: Stop
4: Emergency Stop
*/


//---------------------------------------------------------------------------
// INCLUDES
//---------------------------------------------------------------------------

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <puppeteer_msgs/speed_command.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <float.h>
#include <time.h>
#include <assert.h>
#include <kbhit.h>


//---------------------------------------------------------------------------
// GLOBAL VARIABLES
//---------------------------------------------------------------------------

class KeyboardNode {

private:
  ros::NodeHandle n_;
  ros::ServiceClient client;
  ros::Timer timer;
  puppeteer_msgs::speed_command srv;
  
public:
  KeyboardNode() {
    client = n_.serviceClient<puppeteer_msgs::speed_command>("speed_command");
    timer = n_.createTimer(ros::Duration(0.02), &KeyboardNode::timerCallback, this);
    
    ROS_INFO("Starting Keyboard Node...");
  }

  void timerCallback(const ros::TimerEvent& e) {
    //ROS_DEBUG("timerCallback triggered");
  
    static bool emergency_flag = false;
    int operating_condition = 4;  // initialize operating_condition to emergency stop just for safety
    if(ros::param::has("operating_condition")) {
      ros::param::get("/operating_condition", operating_condition);
      // did we get an emergency stop request?
      if(operating_condition == 4 && emergency_flag == false) {
	ROS_WARN("Emergency Stop Requested");
	emergency_flag = true;
      }
    }
    else {
      ROS_WARN("Cannot Find Parameter: operating_condition");
      ROS_INFO("Setting operating_condition to IDLE");
      ros::param::set("/operating_condition", 0);
      
      return;
    }
    
    // check kbhit() to see if there was a keyboard strike and transfer_flag to see if there is a node sending serial data
    if(kbhit()) {
      ROS_DEBUG("Keyboard Strike Detected");
      
      // get key pressed
      int c = fgetc(stdin);
      
      // what key did we press?
      // did we enter an idle command?
      if(c == 'I') {
	// can we move from our current state to this state?
	if(operating_condition > 1) {
	  ROS_INFO("Robots State: IDLE");
	  ros::param::set("/operating_condition", 0);
	}
	else if(operating_condition == 0) {
	  ROS_INFO("Already in IDLE state");
	}
	else {
	  ROS_INFO("Cannot enter IDLE state from current state: %i", operating_condition);
	}
      }
      
      // did we enter a calibrate command?
      else if(c == 'C')	{
	if(operating_condition < 1) {
	  ROS_INFO("Robots State Change: CALIBRATE");
	  ros::param::set("/operating_condition", 1);
	}
	else if(operating_condition == 1) {
	  ROS_INFO("Already in CALIBRATE state");
	}
	else {
	  ROS_INFO("Cannot enter CALIBRATE state from current state: %i", operating_condition);
	}
      }
      
      // did we enter a run command?
      else if(c == 'P')	{
	if(operating_condition < 2) {
	  ROS_INFO("Robots State Change: RUN");
	  ros::param::set("/operating_condition", 2);
	}
	else if(operating_condition == 2) {
	  ROS_INFO("Already in RUN state");
	}
	else {
	  ROS_INFO("Cannot enter RUN state from current state: %i", operating_condition);
	}
      }

      // did we enter a start string command
      else if(c == 'S')	{
	if(operating_condition < 1) {
	  ROS_INFO("Sending Start String");

	  // check to see if robot index parameter exists.
	  static int robot_index = 0;
	  if(ros::param::has("robot_index")) {
	    ros::param::get("/robot_index", robot_index);
	  }
	  else {
	    ROS_WARN("Cannot Find Parameter: robot_index");
	    ROS_INFO("Setting robot_index to 0");
	    ros::param::set("/robot_index", 0);
	  }
	  
	  // define start string
	  srv.request.robot_index = robot_index;
	  srv.request.type = 'm';
	  srv.request.Vleft = 0.0;
	  srv.request.Vright = 0.0;
	  srv.request.Vtop = 0.0;
	  srv.request.div = 0;
	  
	  // send request to service
	  if(client.call(srv)) {
	    if(srv.response.error == false) {
	      ROS_DEBUG("Send Successful: speed_command\n");
	    }
	    else {
	      ROS_DEBUG("Send Request Denied: speed_command\n");
	      static bool request_denied_notify = true;
	      if(request_denied_notify) {
		ROS_INFO("Send Requests Denied: speed_command\n");
		request_denied_notify = false;
	      }
	    }
	  }
	  else {
	    ROS_ERROR("Failed to call service: speed_command\n");
	  }
	}
	else {
	  ROS_INFO("Cannot send start command from current state: %i", operating_condition);
	}
      }
      
      // did we enter a stop command?
      else {
	if(operating_condition < 3) {
	  ros::param::set("/operating_condition", 3);
	  ROS_INFO("Robots State Change: STOP"); 
	}
	else if(operating_condition == 3 || operating_condition == 4) {
	  ROS_INFO("Already in STOP or EMERGENCY STOP state");
	}
	else {
	  ROS_INFO("Cannot enter STOP state from current state: %i", operating_condition);
	}
      }
    } 
    
    return;
  }
};


//---------------------------------------------------------------------------
// MAIN
//---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_node");

    KeyboardNode keyboard;

    // Wait for new data:
    ros::spin();

    return 0;
 }
