#include <iostream>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Transform.h"
#include <nav_msgs/Odometry.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "gazebo_msgs/ModelStates.h"
#include "sensor_msgs/LaserScan.h"
#include "SDL/SDL.h"
#include <sys/time.h>
#include <fstream>
#include <pthread.h>			//Multi threading concerning WIFI scanning
#include <unistd.h>

//#define PATH "/home/connor/logs/"
#define DTOR(d) ((d) * M_PI / 180)

using namespace std;

float laserRead[181];
bool wifiScan = true;

double ypos=0, xpos=0, theta=0;

/*
 *   <node pkg="robotics_course" type="robotics_course" name="robotics_course_node">
  </node>

 * */

#include <termios.h>
#include <stdio.h>

static struct termios old, new_s;

/* Initialize new terminal i/o settings */
void initTermios(int echo) 
{
  tcgetattr(0, &old); /* grab old terminal i/o settings */
  new_s = old; /* make new settings same as old settings */
  new_s.c_lflag &= ~ICANON; /* disable buffered i/o */
  new_s.c_lflag &= echo ? ECHO : ~ECHO; /* set echo mode */
  tcsetattr(0, TCSANOW, &new_s); /* use these new terminal i/o settings now */
}

/* Restore old terminal i/o settings */
void resetTermios(void) 
{
  tcsetattr(0, TCSANOW, &old);
}

/* Read 1 character - echo defines echo mode */
char getch_(int echo) 
{
  char ch;
  initTermios(echo);
  ch = getchar();
  resetTermios();
  return ch;
}

/* Read 1 character without echo */
char getch(void) 
{
  return getch_(0);
}


int main(int argc, char** argv)
{
	std::cout<<"Program Started\n";
	struct timeval tv;


	double newspeed = 0, newturnrate = 0;
	//ramping robot motor speeds
	double u[2];
	double ulow[] = {0.15, 0.1};
	double uhigh[] = {0.4, 0.25};
	u[0]=ulow[0];
	u[1]=ulow[1];

  	//init the ROS node
	ros::init(argc, argv, "keyboardControl");
	ros::NodeHandle nh;
	ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
	ros::Publisher palm_load_pub = nh.advertise<std_msgs::Float64>("/lift_p3at/palm_joint_position_controller/command", 1000);
	ros::Publisher left_load_pub = nh.advertise<std_msgs::Float64>("/lift_p3at/left_fork_joint_position_controller/command", 1000);
	ros::Publisher right_load_pub = nh.advertise<std_msgs::Float64>("/lift_p3at/right_fork_joint_position_controller/command", 1000);
	ros::Rate rate(10);
	geometry_msgs::Twist base_cmd;
	std_msgs::Float64 load_cmd;
	float basespeed = 0.2;
	while(ros::ok())
	{
        char c = 0;

        switch((c=getch())) {
        case 'w':
            cout << endl << "Up" << endl;//key up
            newspeed = basespeed;
            newturnrate = 0;
            break;
        case 's':
            cout << endl << "Down" << endl;   // key down
            newspeed = -1 * basespeed;
            newturnrate = 0;
            break;
        case 'a':
            cout << endl << "Left" << endl;  // key right
            newspeed = 0;
            newturnrate = -1 * basespeed;
            break;
        case 'd':
            cout << endl << "Right" << endl;  // key left
            newspeed = 0;
            newturnrate = basespeed;
            break;
        case 'l':
            cout << endl << "Loading" << endl;  // key load
            newspeed = 0;
            newturnrate = 0;
			load_cmd.data = 0.2;
			left_load_pub.publish(load_cmd);
			load_cmd.data = 0.15;
			right_load_pub.publish(load_cmd);
			usleep(1000000);
			load_cmd.data = 0.5;
			palm_load_pub.publish(load_cmd);
            
            break;
        case 'u':
            cout << endl << "Unloading" << endl;  // key unload
            newspeed = 0;
            newturnrate = 0;
			load_cmd.data = 0.0;
			palm_load_pub.publish(load_cmd);
			load_cmd.data = 0.0;
			left_load_pub.publish(load_cmd);
			usleep(100000);
			load_cmd.data = 0.7;
			right_load_pub.publish(load_cmd);
			
            break;
        case '+':
			cout<<"High Speed " <<endl;
			basespeed = 0.5;
			break;
        case '-':
			cout<<"Low Speed " <<endl;
			basespeed = 0.2;
			break;			
        case 'X':
            cout << endl << "Exiting" << endl;  // key left
            newspeed = 0;
            newturnrate = 0;
            return 1;
            break;
        default:
            cout << endl << "C " << c << endl;  // not arrow
            newspeed = 0;
            newturnrate = 0;
            break;
        }
		
//		ROS_INFO("newspeed: %f, \t newturn : %f", newspeed, newturnrate);
		base_cmd.linear.x = newspeed;
		base_cmd.angular.z = newturnrate;

		// end loop publish/rate set
		cmd_vel_pub.publish(base_cmd);
		ros::spinOnce();
		rate.sleep();

	}


      return 0;
}
