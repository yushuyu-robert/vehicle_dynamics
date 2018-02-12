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

double timer = 0.01;

double omega_w = 0;
double vb_wheel = 0;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_dynamics");
	ros::NodeHandle n;

	//timer:
	ros::Timer timer_pubstate;

	thisdynamics.T_samp = timer;

	timer_pubstate = n.createTimer(ros::Duration(timer), timerCallback);  //timer used to publish state, should be at least for some minimal frequency


	//subscribe input
	//sub = n.subscribe("input", 1, inputCallback);

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
	//ROS_INFO_STREAM("T_sampling: ");
	test++;// test












	{  //test:

		double tau = 50;
		double pi = 3.14159265;

		    //parameters:
		    double rw = 0.347;
		    double cp = 20;
		    double mass = 2194;
		    double g = 9.8;
		    double theta_g=0;
		    double mu=0.9;
		    double i_wheel = 11;
		    double fr= 0.0164;


		    double sx = -(vb_wheel - rw  * omega_w ) / thisdynamics.max_dynamics(
		    		thisdynamics.abs_dynamics(rw *omega_w ), 0.01);
		    double sxy = thisdynamics.abs_dynamics(sx);
		    double f_sxy  = 2/pi*atan(2*cp*sxy/pi);
		    double Fz = mass*g*cos(theta_g)/2;

		    double Fxy = mu*Fz *f_sxy;
		    double Fw = Fxy*sx/thisdynamics.max_dynamics(sxy,0.1);
		   // double Troll = fr*mass*g*rw;
		    double Troll = 0;

		    double vb_dot = Fw/mass;
		    double omega_dot = (tau-Fw*rw - Troll)/i_wheel;

		    omega_w = omega_dot*timer + omega_w;
		    vb_wheel = vb_dot*timer + vb_wheel;
		    std::cerr << "omega_w: " << omega_w << "  vb_wheel: " << vb_wheel<< std::endl;
		    std::cerr << "vb_dot: " << vb_dot<< std::endl;
		    std::cerr << "omega_dot: " << omega_dot<< std::endl;

		    std::cerr << std::endl;
	}








}


void inputCallback(const vehicle_dynamics::vehicle_inputConstPtr& input){
	thisdynamics.steering_angle = (double)input->steering;
	thisdynamics.A_ped = (double)input->pedal;
	thisdynamics.B_ped = (double)input->brake;
}




