#include "ros/ros.h"
#include "std_msgs/String.h"
#include <stdlib.h>
#include <sstream>
#include "dynamics_vehicle.h"


#define MAX_TREE_SIZE 100
#define PI 3.14159265



dynamics::dynamics(){



}


void dynamics::diff_equation(){
	//the differential equation of all the dynamics

	int i = 0;

	delta[0] = steering_angle;
	delta[1] = 0;


	//wheel
	for (i=0; i<2; i++)
	{

		vb_wheel[0][i] = v_body[0]*cos(delta[i]) + v_body[1] * sin(delta[i]);
		vb_wheel[1][i] = -v_body[0]*sin(delta[i]) + v_body[1] * cos(delta[i]);


		sx[i] = (vb_wheel[0][i] - rw[i] * omega_w[i] ) / max_dynamics(abs_dynamics(rw[i]*omega_w[i]), 0.01);
		sy[i] = (vb_wheel[1][i] ) / max_dynamics(abs_dynamics(rw[i]*omega_w[i]), 0.01);
		sxy[i] = sqrt(sx[i]*sx[i] + sy[i]*sy[i]);

		f_sxy[i] = 2/PI*atan(2*cp*sxy[i]/PI);

		Fxy[i] = mu*Fz[i]*f_sxy[i];
		Fw[0][i] =(rw[i]*omega_w[i] - vb_wheel[0][i])*Fxy[i] / sqrt((rw[i]*omega_w[i] - vb_wheel[0][i])*(rw[i]*omega_w[i]
		             - vb_wheel[0][i]) + vb_wheel[1][i] * vb_wheel[1][i]);
		Fw[1][i] = - vb_wheel[1][i]*Fxy[i] / sqrt((rw[i]*omega_w[i] - vb_wheel[0][i])*(rw[i]*omega_w[i] - vb_wheel[0][i]) + vb_wheel[1][i] * vb_wheel[1][i]);

		T_roll[i] = fr[i]*mass*g*rw[i];



		//force actuated on body, body frame
		Fv[0][i] = cos(delta[i]) * Fw[0][i] - sin(delta[i])*Fw[1][i];
		Fv[1][i] = sin(delta[i]) * Fw[0][i] + cos(delta[i])*Fw[1][i];
	}

 //body:

	F_d[0] = 0.5*rou*C_d*A*v_body[0]*v_body[0];
	F_g[0] = mass*g*sin(theta_g);
	Fx = Fv[0][0]+Fv[0][1] - F_d[0] -F_g[0];

	F_d[1] = 0.5*rou*C_d*A*v_body[1]*v_body[1];
	F_g[1] = mass*g*sin(theta_g);
	Fy = Fv[1][0]+Fv[1][1] - F_d[1] -F_g[1];

	double ax, ay;
	ax = F_d[0]/mass;
	ay = F_d[1]/mass;


	//power strain, rare wheel?
	double omega_d, omega_d_dot;
	omega_d =  omega_w[1];
	omega_d_dot = omega_wheel_dot[1];

	double omega_f, omega_f_dot;
	omega_f = omega_d*i_final;
	omega_f_dot = omega_d_dot*i_final;


	double omega_p, omega_p_dot;
	omega_p = omega_f;
	omega_p_dot = omega_f_dot;

	double omega_t, omega_t_dot;
	omega_t = omega_p * i_gear;
	omega_t_dot = omega_p_dot*i_gear;

	double omega_c, omega_c_dot;
	omega_c = omega_t;
	omega_c_dot = omega_t_dot;

	double omega_e, omega_e_dot;
	omega_e = omega_c;
	omega_e_dot =  omega_c_dot;


	double T_emax;
	T_emax = f_omegae(omega_e);  //3.30


	double Teaped;
	Teaped = A_ped*0.01*T_emax;


	double Te;

	if(vb_dot[0] > a_xupper)
		Te = Efactor*Teaped;
	else if (vb_dot[0] < a_xlower)
		Te = T_emax;
	else
		Te = Teaped*((vb_dot[0]-a_xlower)*(Efactor-1)/(a_xupper - a_xlower)+1);



	//omega_e_dot = (Te-Tc)/Je;
	double Tc;
	Tc = Te - Je*omega_e_dot;

	double Tt = Tc;

	double Tp = eta_tr*Tt*i_gear;

	double Tf = Tp;

	double Td = Tf*eta_fd*i_final;

	double Tw = Td;

	int rc = (r_gear > 0);


	double rev_trq = -1*rc*max_dynamics(Tt,0)*i_gear*i_final;

	if (r_gear == 1)
		T_prop[1] = rev_trq;
	else
		T_prop[1] = max_dynamics(Tw, 0);




	//brake for XC90:
	double T_req = T_bmax*0.01*B_ped;

    //dot of T_b:
	T_b_dot[0] = kb*(T_req-T_b[0]);
	T_b_dot[1] = kb*(T_req-T_b[1]);










	//derivative part:


	for (i=0; i<2; i++){
		//the derivative of the variables:
		omega_wheel_dot[i]= (T_prop[i] - T_brk[i] - Fw[0][i] * rw[i] - T_roll[i])/Iw[i];
	}


	omegab_dot[2] = (lf*Fv[1][0] - lr* Fv[1][1])/Izz;
	vb_dot[1] = (Fv[1][0] + Fv[1][1])/mass - v_body[0] * omega_body[2];

	vb_dot[0] = ax + v_body[1]*omega_body[2];
	vb_dot[1] = ay - v_body[0]*omega_body[2];











}




double dynamics::max_dynamics(double a, double b){
	if (a>=b)
		return a;
	else
		return b;

}

double dynamics::abs_dynamics(double a){
	if (a>=0)
		return a;
	else{
		double b = -a;
		return b;
	}

}

double dynamics::f_omegae(double omega_e){

	return omega_e;

}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "hi");
  ros::NodeHandle n;
   ros::Rate loop_rate(5);

   std_msgs::String msg;
   std::stringstream ss;



//  while (ros::ok())
//  {
//    ros::spinOnce();
//    loop_rate.sleep();
//
//  }


  return 0;
}







