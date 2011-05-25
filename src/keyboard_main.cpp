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
// FUNCTION DECLARATIONS
//---------------------------------------------------------------------------

void timerCallback(const ros::TimerEvent &);

//---------------------------------------------------------------------------
// MAIN
//---------------------------------------------------------------------------

int main(int argc, char** argv)
{
    ros::init(argc, argv, "keyboard_node");
    ros::NodeHandle n;

    // Define the callback function:
    ros::Timer timer = n.createTimer(ros::Duration(0.02), timerCallback);

    ROS_INFO("Starting Keyboard Node...");

    // Wait for new data:
    ros::spin();

    return 0;
 }


//---------------------------------------------------------------------------
// FUNCTIONS
//---------------------------------------------------------------------------

void timerCallback(const ros::TimerEvent& e) {
  //ROS_DEBUG("timerCallback triggered");
  
  static bool emergency_flag = false;
  int operating_condition = 4;  // initialize operating_condition to emergency stop just for safety
  if(ros::param::has("operating_condition"))
    {
      ros::param::get("/operating_condition", operating_condition);
      // did we get an emergency stop request?
      if(operating_condition == 4 && emergency_flag == false)
	{
	  ROS_WARN("Emergency Stop Requested");
	  emergency_flag = true;
	}
    }
  else
    {
      ROS_WARN("Cannot Find Parameter: operating_condition");
      ROS_INFO("Setting operating_condition to IDLE");
      ros::param::set("/operating_condition", 0);
      
      return;
    }

  static bool idle_flag = false;
  static bool calibrate_flag = false;
  static bool run_flag = false;

  // check kbhit() to see if there was a keyboard strike and transfer_flag to see if there is a node sending serial data
  if(kbhit()) 
    {
      ROS_DEBUG("Keyboard Strike Detected");
      int c = fgetc(stdin);
      // what key did we press?
      // did we enter an idle command?
      if(c == 'I')
	{
	  // have we entered this command already?
	  if(idle_flag == false)
	    {
	      // can we move from our current state to this state?
	      if(operating_condition > 1)
		{
		  idle_flag = true;
		  ROS_INFO("Preparing Robots State Change: IDLE");
		  ROS_INFO("Hit 'Enter/Return' to confirm");
		}
	      else if(operating_condition == 0)
		{
		  ROS_INFO("Already in IDLE state");
		}
	      else
		{
		  ROS_INFO("Cannot enter IDLE state from current state: %i", operating_condition);
		}
	    }
	  else
	    {
	      ROS_INFO("Need to Confirm Last Command");
	    }
	}

      // did we enter a calibrate command?
      else if(c == 'C')
	{
	  if(run_flag == false)
	    {
	      if(operating_condition < 1)
		{
		  run_flag = true;
		  ROS_INFO("Preparing Robots State Change: CALIBRATE");
		  ROS_INFO("Hit 'Enter/Return' to confirm");
		}
	      else if(operating_condition == 1)
		{
		  ROS_INFO("Already in CALIBRATE state");
		}
	      else
		{
		  ROS_INFO("Cannot enter CALIBRATE state from current state: %i", operating_condition);
		}
	    }
	  else
	    {
	      ROS_INFO("Need to Confirm Last Command");
	    }
	}

      // did we enter a run command?
      else if(c == 'P')
	{
	  if(run_flag == false)
	    {
	      if(operating_condition < 2)
		{
		  run_flag = true;
		  ROS_INFO("Preparing Robots State Change: RUN");
		  ROS_INFO("Hit 'Enter/Return' to confirm");
		}
	      else if(operating_condition == 2)
		{
		  ROS_INFO("Already in RUN state");
		}
	      else
		{
		  ROS_INFO("Cannot enter RUN state from current state: %i", operating_condition);
		}
	    }
	  else
	    {
	      ROS_INFO("Need to Confirm Last Command");
	    }
	}

      // did we hit return to confirm a command?
      else if(c == 10)
	{
	  if(idle_flag)
	    {
	      ros::param::set("/operating_condition", 0);
	      ROS_INFO("Robots State Change: IDLE"); 
	      idle_flag = false;
	      emergency_flag = false;
	    }
	  else if(calibrate_flag)
	    {
	      ros::param::set("/operating_condition", 1);
	      ROS_INFO("Robots State Change: CALIBRATE"); 
	      calibrate_flag = false;
	    }
	  else if(run_flag)
	    {
	      ros::param::set("/operating_condition", 2);
	      ROS_INFO("Robots State Change: RUN"); 
	      run_flag = false;
	    }
	  else
	    {
	      ROS_INFO("No Command Found to Confirm");
	    }
	}

      // did we enter a stop command?
      else
	{
	  if(operating_condition < 3)
	    {
	      ros::param::set("/operating_condition", 3);
	      ROS_INFO("Robots State Change: STOP"); 
	    }
	  else if(operating_condition == 3 || operating_condition == 4)
	    {
	      ROS_INFO("Already in STOP or EMERGENCY STOP state");
	    }
	  else
	    {
	      ROS_INFO("Cannot enter STOP state from current state: %i", operating_condition);
	    }
	}
    } 
  
  return;
}
