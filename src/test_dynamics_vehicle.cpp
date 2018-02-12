#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <sstream>
#include "dynamics_vehicle.h"


#include <vehicle_dynamics/vehicle_input.h>
#include <geometry_msgs/Twist.h>

void timerCallback(const ros::TimerEvent& event);
void inputCallback(const vehicle_dynamics::vehicle_inputConstPtr& input);

int test;

ros::Publisher vel_pub;
ros::Subscriber sub;

//vehicle dynamics
dynamics thisdynamics;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_dynamics");
	ros::NodeHandle n;

	//timer:
	ros::Timer timer_pubstate;
	double timer = 0.001;
	timer_pubstate = n.createTimer(ros::Duration(timer), timerCallback);  //timer used to publish state, should be at least for some minimal frequency

	//thisdynamics.T_samp = timer;
	//subscribe input
	sub = n.subscribe("input", 1, inputCallback);

	vel_pub = n.advertise<geometry_msgs::Twist>("velocity", 1);

	ros::spin();
}


void timerCallback(const ros::TimerEvent& event){


	thisdynamics.diff_equation();
	thisdynamics.integrator();

	geometry_msgs::Twist msg;

	msg.linear.x = thisdynamics.v_body[0];
	msg.linear.y = thisdynamics.v_body[1];
	msg.angular.z = thisdynamics.omega_body[2];

	msg.angular.x = msg.angular.x+test;

	vel_pub.publish(msg);
	ROS_INFO_STREAM("T_sampling: ");
	test++;// test
}


void inputCallback(const vehicle_dynamics::vehicle_inputConstPtr& input){
	thisdynamics.steering_angle = (double)input->steering;
	thisdynamics.A_ped = (double)input->pedal;
	thisdynamics.B_ped = (double)input->brake;
}






