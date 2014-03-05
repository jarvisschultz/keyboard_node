// keyboard_main.cpp
// Jake Ware and Jarvis Schultz
// Spring 2011


//---------------------------------------------------------------------------
// NOTES
//---------------------------------------------------------------------------

/*
operating_condition parameter
  0: Idle
  1: Calibrate
  2: Run
  3: Stop
  4: Emergency Stop
*/

////////////////
// PUBLISHERS //
////////////////
// operating_condition (puppeteer_msgs/OperatingCondition)

//////////////
// SERVICES //
//////////////
// operating_condition_change (provider) puppeteer_msgs/OperatingConditionChange


//---------------------------------------------------------------------------
// INCLUDES
//---------------------------------------------------------------------------

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <puppeteer_msgs/RobotCommands.h>
#include <puppeteer_msgs/OperatingCondition.h>
#include <puppeteer_msgs/OperatingConditionChange.h>

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
    ros::Publisher serial_pub;
    ros::Publisher state_pub;
    ros::ServiceServer op_change_server;
    ros::Timer timer, show;
    puppeteer_msgs::OperatingCondition op_con_msg;
    int operating_condition;
  
public:
    KeyboardNode() {
	serial_pub = n_.advertise<puppeteer_msgs::RobotCommands>
	    ("/keyboard_serial_commands", 1);
	state_pub = n_.advertise<puppeteer_msgs::OperatingCondition>
	    ("/operating_condition", 1, true);
	op_change_server = n_.advertiseService
	    ("/operating_condition_change", &KeyboardNode::op_change_server_cb, this);
	timer = n_.createTimer(ros::Duration(0.02),
			       &KeyboardNode::timerCallback, this);
	show = n_.createTimer(ros::Duration(0.5),
			      &KeyboardNode::update_screen, this);
    
	ROS_INFO("Starting Keyboard Node...");
	operating_condition = op_con_msg.IDLE;
    }

    void timerCallback(const ros::TimerEvent& e) {
	//ROS_DEBUG("timerCallback triggered");
  
	static bool emergency_flag = false;

	// initialize operating_condition to idle for safety
	if(ros::param::has("/operating_condition")) {
	    ros::param::getCached("/operating_condition", operating_condition);
	    // did we get an emergency stop request?
	    if(operating_condition == 4 && emergency_flag == false) {
		ROS_WARN("Emergency Stop Requested");
		emergency_flag = true;
	    }
	}
	else {
	    ROS_WARN("Cannot Find Parameter: operating_condition");
	    ROS_INFO("Setting operating_condition to IDLE");
	    ros::param::set("/operating_condition", op_con_msg.IDLE);
	    op_con_msg.state = op_con_msg.IDLE;
	    state_pub.publish(op_con_msg);
	    return;
	}
    
	// check kbhit() to see if there was a keyboard strike and
	// transfer_flag to see if there is a node sending serial data
	if(kbhit()) {
	    ROS_DEBUG("Keyboard Strike Detected");
      
	    // get key pressed
	    char c = fgetc(stdin);
	    int robot_index = 0;
      
	    // what key did we press?
	    // did we enter an idle command?
	    if(c == 'I') {
		// can we move from our current state to this state?
		if(operating_condition > 1) {
		    ROS_INFO("Robots State: IDLE");
		    ros::param::set("/operating_condition", op_con_msg.IDLE);
		    op_con_msg.state = op_con_msg.IDLE;
		    state_pub.publish(op_con_msg);
		}
		else if(operating_condition == 0) {
		    ROS_INFO("Already in IDLE state");
		}
		else {
		    ROS_INFO("Cannot enter IDLE state from current state: %i",
			     operating_condition);
		}
	    }
      
	    // did we enter a calibrate command?
	    else if(c == 'C')	{
		if(operating_condition < 1) {
		    ROS_INFO("Robots State Change: CALIBRATE");
		    ros::param::set("/operating_condition", op_con_msg.CALIBRATE);
		    op_con_msg.state = op_con_msg.CALIBRATE;
		    state_pub.publish(op_con_msg);
		}
		else if(operating_condition == 1) {
		    ROS_INFO("Already in CALIBRATE state");
		}
		else
		    ROS_INFO("Cannot enter CALIBRATE state"
			     " from current state: %i", operating_condition);
	    }
      
	    // did we enter a run command?
	    else if(c == 'P')	{
		if(operating_condition < 2) {
		    ROS_INFO("Robots State Change: RUN");
		    ros::param::set("/operating_condition", op_con_msg.RUN);
		    op_con_msg.state = op_con_msg.RUN;
		    state_pub.publish(op_con_msg);
		}
		else if(operating_condition == 2) {
		    ROS_INFO("Already in RUN state");
		}
		else {
		    ROS_INFO("Cannot enter RUN state from"
			     " current state: %i", operating_condition);
		}
	    }

	    // did we enter a start string command
	    else if(c == 'S')	{
		if(operating_condition < 1) {
		    ROS_INFO("Sending Start String");

		    // check to see if robot index parameter exists.
		    if(ros::param::has("/robot_index")) {
			ros::param::get("/robot_index", robot_index);
		    }
		    else {
			ROS_WARN("Cannot Find Parameter: robot_index");
			ROS_INFO("Setting robot_index to 0");
			ros::param::set("/robot_index", 0);
		    }
	  
		    // define start string
		    puppeteer_msgs::RobotCommands srv;
		    srv.robot_index = robot_index;
		    srv.type = (uint8_t) 'm';
		    srv.div = 0;
		    serial_pub.publish(srv);
		}
		else {
		    ROS_INFO("Cannot send start command from "
			     "current state: %i", operating_condition);
		}
	    }

	    else if (isdigit(c))
	    {
		robot_index = atoi(&c);
		ros::param::set("/robot_index", robot_index);
		ROS_INFO("Robot index is now %d",robot_index);
	    }

	    // did we enter a stop command?
	    else
	    {
		if(operating_condition < 3)
		{
		    emergency_flag = false;
		    ros::param::set("/operating_condition", op_con_msg.STOP);
		    op_con_msg.state = op_con_msg.STOP;
		    state_pub.publish(op_con_msg);
		    ROS_INFO("Robots State Change: STOP"); 
		}
		else if(operating_condition == 3 || operating_condition == 4)
		    ROS_INFO("Already in STOP or EMERGENCY STOP state");
		else
		    ROS_INFO("Cannot enter STOP state from current "
			     "state: %i", operating_condition);
	    }
	} 
    
	return;
    }

    bool
    op_change_server_cb(puppeteer_msgs::OperatingConditionChange::Request &req,
			puppeteer_msgs::OperatingConditionChange::Response &resp)
	{
	    uint8_t val = req.state.state;
	    if (val > op_con_msg.EMERGENCY)
	    {
		resp.error = true;
		return false;
	    }
	    else
		resp.error = false;
	    operating_condition = val;
	    ros::param::set("/operating_condition", val);
	    op_con_msg.state = val;
	    state_pub.publish(op_con_msg);
	    return true;
	}
    
    void update_screen(const ros::TimerEvent& e)
	{
	    static int last = 0;

	    if (operating_condition != last)
	    {
		// then update the screen
		switch(operating_condition)
		{
		case 0:
		    ROS_INFO("Operating condition: IDLE");
		    op_con_msg.state = op_con_msg.IDLE;
		    state_pub.publish(op_con_msg);
		    break;
		case 1:
		    ROS_INFO("Operating condition: CALIBRATE");
		    op_con_msg.state = op_con_msg.CALIBRATE;
		    state_pub.publish(op_con_msg);
		    break;
		case 2:
		    ROS_INFO("Operating condition: RUN");
		    op_con_msg.state = op_con_msg.RUN;
		    state_pub.publish(op_con_msg);
		    break;
		case 3:
		    ROS_INFO("Operating condition: STOP");
		    op_con_msg.state = op_con_msg.STOP;
		    state_pub.publish(op_con_msg);
		    break;
		case 4:
		    ROS_INFO("Operating condition: EMERGENCY");
		    op_con_msg.state = op_con_msg.EMERGENCY;
		    state_pub.publish(op_con_msg);
		    break;
		}
	    }
	    last = operating_condition;
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
