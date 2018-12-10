#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <sstream>
#include "dynamics_vehicle.h"
#include <iostream>
#include <fstream>


#include <vehicle_dynamics/vehicle_input.h>
#include <geometry_msgs/Twist.h>

void timerCallback(const ros::TimerEvent& event);
void inputCallback(const vehicle_dynamics::vehicle_inputConstPtr& input);
void eachstep(void);

int test;

ros::Publisher vel_pub;
ros::Subscriber sub;

//vehicle dynamics
dynamics thisdynamics;

double timer = 0.001;


double omega_w = 0;
double vb_wheel = 0;
geometry_msgs::Twist msg;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_dynamics");
	ros::NodeHandle n;
	vel_pub = n.advertise<geometry_msgs::Twist>("velocity", 1);

	//timer:
	ros::Timer timer_pubstate;
	//timer_pubstate = n.createTimer(ros::Duration(timer), timerCallback);  //timer used to publish state, should be at least for some minimal frequency


	thisdynamics.T_samp = timer;

        double duration = 2000;

        while(thisdynamics.T_global <= 20 ){
		thisdynamics.SetAcceleratorPedalPosition(5*thisdynamics.T_global);
	    eachstep();
	}

	while((thisdynamics.T_global > 20 )&& (thisdynamics.T_global < duration)){
			thisdynamics.SetAcceleratorPedalPosition(0);
			thisdynamics.SetBrakePedalPosition(50);
		    eachstep();
	}


	std::cerr << "finished " << std::endl;



	//subscribe input
	//sub = n.subscribe("input", 1, inputCallback);



	ros::spin();
}


void timerCallback(const ros::TimerEvent& event){

	eachstep();

}


void inputCallback(const vehicle_dynamics::vehicle_inputConstPtr& input){
	thisdynamics.input_global.steering_angle = (double)input->steering;
	thisdynamics.input_global.A_ped = (double)input->pedal;
	thisdynamics.input_global.B_ped = (double)input->brake;
}


void eachstep(void){

	thisdynamics.integrator();



	msg.linear.x = thisdynamics.state_global.v_body[0];
	msg.linear.y = thisdynamics.state_global.v_body[1];
	msg.angular.z = thisdynamics.state_global.omega_body[2];

	//msg.angular.x = msg.angular.x+test;

	vel_pub.publish(msg);
	//ROS_INFO_STREAM("T_sampling: ");
	test++;// test


//	{  //test:
//
//		double tau = 50;
//		double pi = 3.14159265;
//
//		    //parameters:
//		    double rw = 0.347;
//		    double cp = 20;
//		    double mass = 2194;
//		    double g = 9.8;
//		    double theta_g=0;
//		    double mu=0.9;
//		    double i_wheel = 11;
//		    double fr= 0.0164;
//
//
//		    double sx = -(vb_wheel - rw  * omega_w ) / thisdynamics.max_dynamics(
//		    		thisdynamics.abs_dynamics(rw *omega_w ), 0.01);
//		    double sxy = thisdynamics.abs_dynamics(sx);
//		    double f_sxy  = 2/pi*atan(2*cp*sxy/pi);
//		    double Fz = mass*g*cos(theta_g);
//
//		    double Fxy = mu*Fz *f_sxy;
//		    double Fw = Fxy*sx/thisdynamics.max_dynamics(sxy,0.1);
//		   // double Troll = fr*mass*g*rw;
//		    double Troll = 0;
//
//		    double vb_dot = Fw/mass;
//		    double omega_dot = (tau-Fw*rw - Troll)/i_wheel;
//
//		    omega_w = omega_dot*timer + omega_w;
//		    vb_wheel = vb_dot*timer + vb_wheel;
////		    std::cerr << "omega_w: " << omega_w << "  vb_wheel: " << vb_wheel<< std::endl;
////		    std::cerr << "vb_dot: " << vb_dot<< std::endl;
////		    std::cerr << "omega_dot: " << omega_dot<< std::endl;
//
//		    std::cerr << std::endl;
//	}

}



