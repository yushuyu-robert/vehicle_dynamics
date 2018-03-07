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



typedef struct
{
	double v_body[3];
	double omega_body[3];  //angular velocity, body frame
	//the angular velocity of the wheel
	double omega_w[2];
	//braking torque:
	double T_b_general;
	double Ttop;
	double T_new_req;
} state_vehicle;

typedef struct
{
	double vb_dot[3];  //dot of velocity of body
	//the derivative of angular velocity of the wheel
	double omegab_dot[3];  //dot of angular velocity of body
	double omega_wheel_dot[2];
	double T_b_dot_general; //dot of T_b
	double Ttop_dot;
	double T_new_req_dot;
} diff_vehicle;

typedef struct
{
	double A_ped;   //pedal
	double B_ped;  //brake
	double steering_angle;
} input_vehicle;


class dynamics{
public:
	dynamics();
	//the functions:
	void diff_equation(state_vehicle &x, input_vehicle &u,  double t, diff_vehicle &out);
	void integrator(void);
	double max_dynamics(double a, double b);
	double min_dynamics(double a, double b);
	double abs_dynamics(double a);
	double CalcEngineMaxTorque(double m_engineSpeed);

	//interface functions of body:
	double GetLateralAcceleration() const;
	double GetLateralVelocity() const;
	double GetLongitudinalAcceleration() const;
	double GetLongitudinalVelocity() const;
	double GetYawAcceleration() const;
	double GetYawVelocity() const;

	//interface functions of power:
	double GetAcceleratorPedalPosition() const;
	double GetEngineSpeed() const;
	double GetEngineTorque() const;
	int32_t GetGear() const;
	void SetAcceleratorPedalPosition(double);

	double GetBrakePedalPosition() const;
	void SetBrakePedalPosition(double);

	//interface functions of wheel:
	double GetFrontWheelSpeed() const;
	double GetRearWheelSpeed() const;
	double GetRoadWheelAngle() const;
	void SetRoadWheelAngle(double);

	state_vehicle state_global;
	diff_vehicle diff_global;
	input_vehicle input_global;

	///////////////////the parameters of the vehicle//////////////////////
	double PI;
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
	int r_gear;
	double i_tm[12]; //gear ration for each gear


	//upper and lower bounds of acceleration limits,
	double a_xupper;
	double a_xlower;

	//relating to brake:
	double T_bmax;
	double kb;

	//  omega_wheel_dot = (T_prop - T_brk - Fw[0] * rw - T_roll);
	double T_prop[2];
	double T_brk[2];

////others:
	//time step:
	double T_samp;
	double T_global;

	int agear;
	int agear_diff;

private:
	double omega_e;
	double Te;
};

#endif /* DYNAMICS_VEHICLE_H_ */
