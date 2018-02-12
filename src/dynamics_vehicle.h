/*
 * dynamics_vehicle.h
 *
 *  Created on: Jan 22, 2018
 *      Author: yushu
 */

#ifndef DYNAMICS_VEHICLE_H_
#define DYNAMICS_VEHICLE_H_

//other header files

#include "std_msgs/String.h"



//from 22nd, Jan., 2017,  the dynamics of the vehicles

class dynamics{
public:
	dynamics();

	//the functions:
	void diff_equation(void);
	void integrator(void);
	double max_dynamics(double a, double b);
	double abs_dynamics(double a);
	double CalcEngineMaxTorque(double m_engineSpeed);

	///////////////////the parameters of the vehicle//////////////////////

	//relate to wheels
	double rw[2];  //the radius of the wheel
	double cp;  //parameter of cornering stiffness
	double mu;   //friction coefficient
	double Iw[2]; //the inertia of the wheel
	double fr[2];  //rolling resistance coefficient


	double mass; //mass
	double g;  //acc due to gravity
	double rou;
	double C_d;
	double A;
	double theta_g;
	double lf;
	double lr;
	double Izz;


	double Je;

	//the fraction by which the engine torque is reduced
	double Efactor;

	double i_final;
	double i_gear;
	double eta_tr;
	double eta_fd;
	double r_gear;
	double i_tm[10]; //gear ration for each gear
	double velocity_bound[2][10];

	//upper and lower bounds of acceleration limits,
	double a_xupper;
	double a_xlower;

	//relating to brake:
	double T_bmax;
	double kb;

	/////////////////////////////input//////////////////////////////
	double A_ped;   //pedal
	double B_ped;  //brake
	double steering_angle;



	/////////////////////////states/////////////////////////////////////
	//the velocity of the vehicle, expressed in the body frame of the vehicle
	double v_body[3];
	double omega_body[3];  //angular velocity, body frame

	double vb_dot[3];  //dot of velocity of body
	double omegab_dot[3];  //dot of angular velocity of body

	//the angular velocity of the wheel
	double omega_w[2];
	//the derivative of angular velocity of the wheel
	double omega_wheel_dot[2];

	//  omega_wheel_dot = (T_prop - T_brk - Fw[0] * rw - T_roll);
	double T_prop[2];
	double T_brk[2];

	//braking torque:
	double T_b_general;
	double T_b_dot_general; //dot of T_b

	//time step:
	double T_samp;

	int agear;
	int agear_diff;

private:

};

#endif /* DYNAMICS_VEHICLE_H_ */
