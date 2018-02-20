
#include "dynamics_vehicle.h"






dynamics::dynamics(){

	///////////////////the parameters of the vehicle//////////////////////

	PI = 3.14159265;
	//relate to wheels
	rw[0] = 0.435; rw[1] = 0.435;  //the radius of the wheel, FH16
	cp = 20;  //parameter of cornering stiffness
	mu = 0.9;   //friction coefficient
	Iw[0] = 11; Iw[1] = 11; //the inertia of the wheel, FH16
	fr[0] = 0.0164; fr[1] = 0.0164;  //rolling resistance coefficient, FH16

	mass = 9840; //mass, FH16
	g = 9.8;  //acc due to gravity
	rou = 1.225;  C_d = 0.7;  A = 10;//coefficient of air drag, FH16

	theta_g = 0; //slope
	lf = 1.68;  //FH16
	lr = 1.715;   //FH16
	Izz = 41340;  //FH16

	Je = 4;  //flywheel inertia, FH16

	//the fraction by which the engine torque is reduced
	Efactor = 0.5; ////fh16

	i_final = 3.46;  //FH16

	//fh16:
	i_tm[0] = 11.73; //gear ration for each gear
	i_tm[1] = 9.20; //gear ration for each gear
	i_tm[2] = 7.09; //gear ration for each gear
	i_tm[3] = 5.57; //gear ration for each gear
	i_tm[4] = 4.35; //gear ration for each gear
	i_tm[5] = 3.41; //gear ration for each gear
	i_tm[6] = 2.7; //gear ration for each gear
	i_tm[7] = 2.12; //gear ration for each gear
	i_tm[8] = 1.63; //gear ration for each gear
	i_tm[9] = 1.28; //gear ration for each gear
	i_tm[10] = 1.00; //gear ration for each gear
	i_tm[11] = 0.79; //gear ration for each gear


	//XC90
//	i_tm[0] = 5.2; //gear ration for each gear
//	i_tm[1] = 3.029; //gear ration for each gear
//	i_tm[2] = 1.96; //gear ration for each gear
//	i_tm[3] = 1.469; //gear ration for each gear
//	i_tm[4] = 1.231; //gear ration for each gear
//	i_tm[5] = 1; //gear ration for each gear
//	i_tm[6] = 0.809; //gear ration for each gear
//	i_tm[7] = 0.673; //gear ration for each gear
	//5.2,3.029,1.96,1.469,1.231,1,0.809,0.673
	// velocity_bound[2][10];

	i_gear = i_tm[0];
	eta_tr = 1;  //efficient, FH16
	eta_fd = 0.9; //efficient, FH16
	r_gear = 0; //the flag of backward

	//upper and lower bounds of acceleration limits,
	a_xupper = 1.35; //fh16
	a_xlower = 1.1;  //fh16

	//relating to brake:
	T_bmax = 0;
	kb = 0;

	T_prop[0] = 0;  T_prop[1] = 0;
	T_brk[0] = 0;   T_brk[1] = 0;

	/////////////////////////////input//////////////////////////////
	input_global.A_ped = 0;   //pedal
	input_global.B_ped = 0;  //brake
	input_global.steering_angle = 0;

	/////////////////////////states/////////////////////////////////////
	//the velocity of the vehicle, expressed in the body frame of the vehicle
	for (int i = 0; i < 3; i++){
		state_global.v_body[i] = 0;
		state_global.omega_body[i] = 0;  //angular velocity, body frame

		diff_global.vb_dot[i] = 0;  //dot of velocity of body
		diff_global.omegab_dot[i] = 0;  //dot of angular velocity of body
	}

	for (int i = 0; i < 2; i++){
	//the angular velocity of the wheel
		state_global.omega_w[i] = 0;
	//the derivative of angular velocity of the wheel
		diff_global.omega_wheel_dot[i] = 0;

	}
	//braking torque:
	state_global.T_b_general = 0;
	diff_global.T_b_dot_general = 0; //dot of T_b
	state_global.T_new_req = 0;
	state_global.Ttop = 0;



	//time step:
	T_samp = 1;
	T_global = 0;

	agear = 6;
	agear_diff = 0;

}

void dynamics::te(int x){

}


void dynamics::diff_equation(state_vehicle &state, input_vehicle &input,  double t_sim, diff_vehicle &out){
	//the differential equation of all the dynamics

	int i = 0;
	//input:

	double A_ped = input.A_ped;   //pedal
	double B_ped = input.B_ped;  //brake
	double steering_angle = input.steering_angle;

	/////////////////////////states/////////////////////////////////////
	//the velocity of the vehicle, expressed in the body frame of the vehicle
	double v_body[3];
	double omega_body[3];  //angular velocity, body frame
	//the angular velocity of the wheel
	double omega_w[2];
	//braking torque:
	double T_b_general;
	double Ttop;
	double T_new_req;


//	double vb_dot[3];  //dot of velocity of body
//	//the derivative of angular velocity of the wheel
//	double omegab_dot[3];  //dot of angular velocity of body
//	double omega_wheel_dot[2];
//	double T_b_dot_general; //dot of T_b

	//state:
	for (i = 0; i < 3; i++){
		v_body[i] = state.v_body[i];
		omega_body[i] = state.omega_body[i];  //angular velocity, body frame
	}
	for (i = 0; i < 2; i++){
			//the angular velocity of the wheel
		omega_w[i] = state.omega_w[i];
	}
			//braking torque:
		T_b_general = state.T_b_general;
		Ttop = state.Ttop;
		T_new_req = state.T_new_req;


	//wheel
	double f_sxy[2];  //3.14, middle variable
	double Fxy[2];  //3.15, middle variable
	double Fz[2];  //force along the z direction
	double Fw[3][2];  //force expressed in wheel frame
	double Fv[3][2];  //force expressed in body frame
	double T_roll[2];
	//the velocity of the vehicle, expressed in the wheel frame
	double vb_wheel[3][2];
	//slip
	double sx[2];
	double sy[2];
	double sxy[2];

	double delta[2];  //the steering angle
	delta[0] = steering_angle;
	delta[1] = 0;
	for (i=0; i<2; i++){
		vb_wheel[0][i] = v_body[0]*cos(delta[i]) + v_body[1] * sin(delta[i]);
		vb_wheel[1][i] = -v_body[0]*sin(delta[i]) + v_body[1] * cos(delta[i]);

		sx[i] = -(vb_wheel[0][i] - rw[i] * omega_w[i] ) / max_dynamics(abs_dynamics(rw[i]*omega_w[i]), 0.01);
		sy[i] = -(vb_wheel[1][i] ) / max_dynamics(abs_dynamics(rw[i]*omega_w[i]), 0.01);
		sxy[i] = sqrt(sx[i]*sx[i] + sy[i]*sy[i]);

		f_sxy[i] = 2/PI*atan(2*cp*sxy[i]/PI);
		Fz[i] = mass*g*cos(theta_g)/2;
		Fxy[i] = mu*Fz[i]*f_sxy[i];
//		Fw[0][i] =(rw[i]*omega_w[i] - vb_wheel[0][i])*Fxy[i] / sqrt((rw[i]*omega_w[i] - vb_wheel[0][i])*(rw[i]*omega_w[i]
//		             - vb_wheel[0][i]) + vb_wheel[1][i] * vb_wheel[1][i]);
//		Fw[1][i] = - vb_wheel[1][i]*Fxy[i] / sqrt((rw[i]*omega_w[i] - vb_wheel[0][i])*(rw[i]*omega_w[i] - vb_wheel[0][i]) + vb_wheel[1][i] * vb_wheel[1][i]);

		Fw[0][i] = Fxy[i]*sx[i]/max_dynamics(sxy[i],0.1);
		Fw[1][i] = Fxy[i]*sy[i]/max_dynamics(sxy[i],0.1);

		if (omega_w[i] > 0.00000){
			T_roll[i] = fr[i]*mass*g*rw[i]/2;
		}
		else if (omega_w[i] < -0.00000){
			T_roll[i] = -fr[i]*mass*g*rw[i]/2;
		}
		else
			T_roll[i] = 0;

		//T_roll[i] = 0;  //test

		//force actuated on body, body frame
		Fv[0][i] = cos(delta[i]) * Fw[0][i] - sin(delta[i])*Fw[1][i];
		Fv[1][i] = sin(delta[i]) * Fw[0][i] + cos(delta[i])*Fw[1][i];
	}

	//brake for XC90:
	double T_req = T_bmax*0.01*B_ped;

	//test:
	T_brk[0] = T_b_general/2;
	T_brk[1] = T_b_general/2;

	T_brk[0] = 0;
	T_brk[1] = 0;


	//body:
	double F_d[2];
	double F_g[2];
	double Fx; //total force
	double Fy;

	//x direction, body frame

	if(v_body[0] > 0.00){
		F_d[0] = 0.5*rou*C_d*A*v_body[0]*v_body[0];
		F_g[0] = mass*g*sin(theta_g);
	}
	else if(v_body[0] < -0.00){
		F_d[0] = -0.5*rou*C_d*A*v_body[0]*v_body[0];
		F_g[0] = mass*g*sin(theta_g);
	}
	else{
		F_d[0] = 0;
		F_g[0] = mass*g*sin(theta_g);
	}
	Fx = Fv[0][0] + Fv[0][1] - F_d[0] -F_g[0];

	//Fx = Fv[0][0] + Fv[0][1]; //test



	//y direction, body frame
//	F_d[1] = 0.5*rou*C_d*A*v_body[1]*v_body[1];
//	F_g[1] = mass*g*sin(theta_g);
//	Fy = Fv[1][0]+Fv[1][1] - F_d[1] -F_g[1];
	Fy = Fv[1][0] + Fv[1][1];

	double ax, ay;
	ax = Fx/mass;
	ay = Fy/mass;

	int drive_flag = 1;  // 1: rear driving, 0: rear and front driving
	//power strain, XC90:
	double omega_d, omega_d_dot;

	if(drive_flag == 0)
	{
		omega_d =  (omega_w[0] + omega_w[1])/2;
		omega_d_dot = (diff_global.omega_wheel_dot[0] + diff_global.omega_wheel_dot[1])/2;
	}

	if(drive_flag == 1){
		omega_d =  omega_w[1];
		omega_d_dot =   diff_global.omega_wheel_dot[1];
	}


	double omega_f, omega_f_dot;
	omega_f = omega_d*i_final;
	omega_f_dot = omega_d_dot*i_final;


	double omega_p, omega_p_dot;
	omega_p = omega_f;
	omega_p_dot = omega_f_dot;

	double omega_t, omega_t_dot;
	i_gear = i_tm[agear-1];  //agear is from 1 t0 ...,
	//i_gear = i_tm[0];  //test
	omega_t = omega_p * i_gear;
	omega_t_dot = omega_p_dot*i_gear;

	double omega_c, omega_c_dot;
	omega_c = omega_t;
	omega_c_dot = omega_t_dot;

	double omega_e, omega_e_dot;
	omega_e = omega_c;
	omega_e_dot =  omega_c_dot;


	double T_emax;
	T_emax = CalcEngineMaxTorque(omega_e);  //3.30


	A_ped = 10; //test
  //  A_ped = T_global * 10;
	double Teaped;
	Teaped = A_ped*0.01*T_emax;

	if ((omega_e == 0) &&(A_ped >0 ))
	{
		Teaped=1;
		T_emax = 1;
	}

	double T_alim;

	if(diff_global.vb_dot[0] > a_xupper)
		T_alim = Efactor*Teaped;
	else if (diff_global.vb_dot[0] < a_xlower)
		T_alim = Teaped;
	else
		T_alim = Teaped*((diff_global.vb_dot[0]-a_xlower)*(Efactor-1)/(a_xupper - a_xlower)+1);


	double Te;

	out.T_new_req_dot = Teaped - T_new_req;

	double T_req_alim = min_dynamics(T_new_req, T_alim);
	double Tsplit = 700;
	double Tbase = min_dynamics(T_req_alim, Tsplit);
	double Tdynreq = T_req_alim - Tbase;
	double k = 0.5;
	out.Ttop_dot = k*(Tdynreq - Ttop);
	Te = Tbase + Ttop;

	//Te = 700;

	if(diff_global.vb_dot[0] > a_xupper)
		Te = Efactor*Teaped;
	else if (diff_global.vb_dot[0] < a_xlower)
		Te = T_emax;
	else
		Te = Teaped*((diff_global.vb_dot[0]-a_xlower)*(Efactor-1)/(a_xupper - a_xlower)+1);

	//T_emax = 700;
	//Te = T_emax;

	//omega_e_dot = (Te-Tc)/Je;
	double Tc;
	Tc = Te - Je*abs_dynamics(omega_e_dot);
	Tc = Te;

	//Tc = Te - Je*(omega_e_dot);

//	Te = 100;
//	Tc = Te; //test
//	Tc = Te - Je*omega_e_dot;

	double k_speed_wtoe = i_final*i_gear;
	double	k_tau_ctow = eta_tr*i_gear*eta_fd*i_final;
	double f2[2];
	for (i=0; i<2; i++){
    	f2[i] =  Fw[0][i] * rw[i] + T_roll[i] + T_brk[i];
	}
	//Tc = (Iw[1] * Te + k_speed_wtoe*Je*f2[1])/(Iw[1]  + k_speed_wtoe*Je*k_tau_ctow); //according to Je*\dot omega_e = Te - Tc
	//Tc = Te;


	double Tt = Tc;
	double Tp = eta_tr*Tt*i_gear;
	double Tf = Tp;
	double Td = Tf*eta_fd*i_final;

	//XC90:
	double Twf;
	double Twr;

	//int drive_flag = 1;  // 1: rear driving, 0: rear and front driving
	if(drive_flag == 0){
		Twf = 0.4*Td;
		Twr = 0.6*Td;
	}
	if(drive_flag == 1){
		Twf = 0;
		Twr = 1*Td;
	}

	int rc = (r_gear > 0);

	double rev_trq = -1*rc*max_dynamics(Tt,0)*i_gear*i_final;

	if (r_gear == 1){
		T_prop[0] = 0.4*rev_trq;
		T_prop[1] = 0.6*rev_trq;
	}
	else{
		T_prop[0] = max_dynamics(Twf, 0);
		T_prop[1] = max_dynamics(Twr, 0);
	}

//	T_prop[0] = 50; //test
//	T_prop[1] = 50;  //test


	//derivative part:

	double Te_direct[2];
	Te_direct[0] = 0;
	Te_direct[1] = Te;

	//derivative of wheel rotational velocity
	for (i=0; i<2; i++){
		//T_roll[i] = 0; //test

		double forceinducedtorque;
//		if (omega_w[i]>0.01){
//			forceinducedtorque = Fw[0][i] * rw[i];
//		}
//		else {
//			forceinducedtorque = Fw[0][i] * rw[i];
//		}

		forceinducedtorque = Fw[0][i] * rw[i];

		//out.omega_wheel_dot[i]= (T_prop[i] - T_brk[i] - forceinducedtorque - T_roll[i])/Iw[i];

     	//Je = 0; //if the flywheel is not considered
		out.omega_wheel_dot[i]=  (Te_direct[i] - 1/(k_tau_ctow)*(T_brk[i] + forceinducedtorque + T_roll[i] ))
				/(Je*k_speed_wtoe + Iw[i]/k_tau_ctow);

	}

	//if the powerstrain is considered:


	//derivative of body rotational velocity
	out.omegab_dot[2] = (lf*Fv[1][0] - lr* Fv[1][1])/Izz;

	//derivative of body velocity, expressed in body frame:
	out.vb_dot[0] = ax + v_body[1]*omega_body[2];
	out.vb_dot[1] = ay - v_body[0]*omega_body[2];
    //dot of T_b:
	out.T_b_dot_general = kb*(T_req-T_b_general);


	//agear
	double velocity_bound[2][12] =
	{{2.31,3,3.81,4.05,4.25,5.05,8.25,9.85,13.45,15.78,21.1,1000000},
	 {-1000000,2.2,2.28,3.5,3.9,4.1,4.77,6.1,6.5,8.5, 14.5,20}};

	if(mass > 12000){

		if ((v_body[0] > velocity_bound[0][agear-1]) && (agear <=11) )//upper bound
			agear_diff = 1;
		else if ( (v_body[0] < velocity_bound[1][agear-1]) && (agear >= 2))  //lower bound
			agear_diff = -1;
		else
			agear_diff = 0;
	}
	else{
		if ((v_body[0] > velocity_bound[0][agear-1]) && (agear <=11) )//upper bound
			agear_diff = 1;
		else if ( (v_body[0] < velocity_bound[1][agear-1]) && (agear >= 7))  //lower bound
			agear_diff = -1;
		else
			agear_diff = 0;
	}






	////////test only, Feb. 12///
//    //parameters:
//    rw[0] = 0.347;
//    cp = 20;
//    mass = 2194;
//    g = 9.8;
//    theta_g=0;
//    mu=0.9;
//    double i_wheel = 11;
//    fr[0]= 0.0164;
//
//
//    sx[0] = -(v_body[0] - rw[0]  * omega_w[0] ) / max_dynamics(abs_dynamics(rw[0] *omega_w[0] ), 0.01);
//    sxy[0] = abs_dynamics(sx[0]);
//    f_sxy[0] = 2/PI*atan(2*cp*sxy[0]/PI);
//    double Fzz = mass*g*cos(theta_g)/2;
//    double Ftest = mu*Fzz *f_sxy[0];
//    Fw[0][0] = Ftest*sx[0]/max_dynamics(sxy[0],0.1);
//
//    out.vb_dot[0] = Fw[0][0]/mass;
//    out.omega_wheel_dot[0] = (50-Fw[0][0]*rw[0] )/i_wheel;

    /////////////////test finish/////////////////



	//ROS_INFO_STREAM("received path commands, flag_pc_cmd is set to)"<<vb_dot[0]));


//	std::cerr << "omega_w[0]: " << omega_w[0]  << "  omega_w[1]: " << omega_w[1] << std::endl;
//
//	std::cerr << "v_body[0]: " << v_body[0]
//	                                     << "	v_body[1]: " << v_body[1]
//<< "	omega_body[2]: " << omega_body[2] << std::endl;


//	std::cerr << "omega_wheel_dot[0]: " << out.omega_wheel_dot[0] <<
//			"  omega_wheel_dot[1]: " << out.omega_wheel_dot[1] << std::endl;
//	std::cerr << "vb_dot[0]: " << out.vb_dot[0] <<
//			"  vb_dot[1]: " << out.vb_dot[1] << std::endl;
//	std::cerr << "omegab_dot[2]: " << omegab_dot[2] << std::endl;
////
//
//	std::cerr << "Fw[0][0]: " << Fw[0][0]
//	          << "  Fw[1][0]: " << Fw[1][0]
//      << "   Fw[0][1]: " << Fw[0][1]<<
//      "   Fw[1][1]: " << Fw[1][1] << std::endl;
//
//	std::cerr << "sx[0]:" << sx[0]
//			<< "	sx[1]:" << sx[1]
//         << "	sy[0]:" << sy[0]
//           << "	sy[1]:" << sy[1] << std::endl;
//
	std::cerr << "Tc:" << Tc << "	Te:" << Te << "	Temx:" << T_emax << "  speed of engine:" << omega_e
			<<"  speed:" << omega_f<<std::endl;
//
//	std::cerr << "T_emax:" << T_emax << std::endl;
//
//	std::cerr << "T_prop[0]: " << T_prop[0]
// << "	T_prop[1]: " << T_prop[1]
//<< "	T_brk[0]: " << T_brk[0]
// << "	T_brk[1]: " << T_brk[1]
//<< "	T_roll[0]: " << T_roll[0]
//<< "	T_roll[1]: " << T_roll[1] << std::endl;
//





}



void dynamics::integrator(void){

	//update stete:
	int flag = 0;
	switch (flag)
	{
		{
			//4-order:
		case 0:
			state_vehicle x1, x2, x3, x4;
			diff_vehicle k1, k2, k3, k4;

			diff_equation(state_global, input_global,  T_global, k1);

			x2.T_b_general = state_global.T_b_general+ 0.5*T_samp*k1.T_b_dot_general;
			x2.Ttop = state_global.Ttop+ 0.5*T_samp*k1.Ttop_dot;
			x2.T_new_req = state_global.T_new_req+ 0.5*T_samp*k1.T_new_req_dot;
			for(int i = 0; i < 3; i++){
				x2.v_body[i] = state_global.v_body[i] + 0.5*k1.vb_dot[i]*T_samp;
				x2.omega_body[i] = state_global.omega_body[i] + 0.5*k1.omegab_dot[i]*T_samp;
			}
			for(int j = 0; j < 2; j++){
				x2.omega_w[j] = 0.5*k1.omega_wheel_dot[j]*T_samp + state_global.omega_w[j];
			}
			diff_equation(x2, input_global,  T_global+0.5*T_samp, k2);

			x3.T_b_general = state_global.T_b_general+ 0.5*T_samp*k2.T_b_dot_general;
			x3.Ttop = state_global.Ttop+ 0.5*T_samp*k2.Ttop_dot;
			x3.T_new_req = state_global.T_new_req+ 0.5*T_samp*k2.T_new_req_dot;
			for(int i = 0; i < 3; i++){
				x3.v_body[i] = state_global.v_body[i] + 0.5*k2.vb_dot[i]*T_samp;
				x3.omega_body[i] = state_global.omega_body[i] + 0.5*k2.omegab_dot[i]*T_samp;
			}
			for(int j = 0; j < 2; j++){
				x3.omega_w[j] = 0.5*k2.omega_wheel_dot[j]*T_samp + state_global.omega_w[j];
			}
			diff_equation(x3, input_global,  T_global + 0.5*T_samp, k3);

			x4.T_b_general = state_global.T_b_general+ T_samp*k3.T_b_dot_general;
			x4.Ttop = state_global.Ttop+ T_samp*k3.Ttop_dot;
			x4.T_new_req = state_global.T_new_req+  T_samp*k3.T_new_req_dot;
			for(int i = 0; i < 3; i++){
				x4.v_body[i] = state_global.v_body[i] + k3.vb_dot[i]*T_samp;
				x4.omega_body[i] = state_global.omega_body[i] + k3.omegab_dot[i]*T_samp;
			}
			for(int j = 0; j < 2; j++){
				x4.omega_w[j] = k3.omega_wheel_dot[j]*T_samp + state_global.omega_w[j];
			}
			diff_equation(x4, input_global,  T_global + T_samp, k4);


			state_global.T_b_general = state_global.T_b_general+ T_samp*(k1.T_b_dot_general +
			2*k2.T_b_dot_general + 2*k3.T_b_dot_general+ k4.T_b_dot_general)/6;
			state_global.Ttop = state_global.Ttop+ T_samp*(k1.Ttop_dot +
					2*k2.Ttop_dot + 2*k3.Ttop_dot+ k4.Ttop_dot)/6;
			state_global.T_new_req = state_global.T_new_req+  T_samp*(k1.T_new_req_dot +
					2*k2.T_new_req_dot + 2*k3.T_new_req_dot+ k4.T_new_req_dot)/6;
			for(int i = 0; i < 3; i++){
				state_global.v_body[i] = state_global.v_body[i] + T_samp*(
						k1.vb_dot[i] + 2*k2.vb_dot[i] + 2*k3.vb_dot[i] + k4.vb_dot[i])/6;
				state_global.omega_body[i] = state_global.omega_body[i] + (
						k1.omegab_dot[i] + 2* k2.omegab_dot[i] + 2*k3.omegab_dot[i] + k4.omegab_dot[i])*T_samp/6;
			}
			for(int j = 0; j < 2; j++){
				state_global.omega_w[j] = state_global.omega_w[j] +  (
						k1.omega_wheel_dot[j] + 2*k2.omega_wheel_dot[j] + 2* k3.omega_wheel_dot[j] + k4.omega_wheel_dot[j])*T_samp/6;
			}

			T_global = T_global+T_samp;
			diff_global.vb_dot[0] = ( k1.vb_dot[0] + 2*k2.vb_dot[0] + 2*k3.vb_dot[0] + k4.vb_dot[0])/6;
			diff_global.omegab_dot[0] = ( k1.omegab_dot[0] + 2*k2.omegab_dot[0] + 2*k3.omegab_dot[0] + k4.omegab_dot[0])/6;
			diff_global.omegab_dot[1] = ( k1.omegab_dot[1] + 2*k2.omegab_dot[1] + 2*k3.omegab_dot[1] + k4.omegab_dot[1])/6;
			diff_global.omega_wheel_dot[1] = ( k1.omega_wheel_dot[1] +
					2*k2.omega_wheel_dot[1] + 2*k3.omega_wheel_dot[1] + k4.omega_wheel_dot[1])/6;

			break;
		}



		{ //1-order:
		case 1:

			diff_vehicle k_1st;
			diff_equation(state_global, input_global,  T_global, k_1st);
			state_global.T_b_general = state_global.T_b_general + T_samp*k_1st.T_b_dot_general;
			//body:
			for(int i = 0; i < 3; i++){
				state_global.v_body[i] = state_global.v_body[i] + k_1st.vb_dot[i]*T_samp;
				state_global.omega_body[i] = state_global.omega_body[i] + k_1st.omegab_dot[i]*T_samp;
			}
			//wheel:
			for(int j = 0; j < 2; j++){
				state_global.omega_w[j] = k_1st.omega_wheel_dot[j]*T_samp + state_global.omega_w[j];
			}
			T_global = T_global+T_samp;
			break;
		}

	}
	//agear
	agear = agear_diff + agear;

 	std::cerr << "angular velocity of wheel: " << state_global.omega_w[0]  << "," <<  state_global.omega_w[1] << std::endl;
////	std::cerr << "angular acc of wheel:  " << diff_global.omega_wheel_dot[0] << ","  << diff_global.omega_wheel_dot[1] << std::endl;
//
	std::cerr << "body velocity (x, y, rot z): " << state_global.v_body[0] << ","
			<< state_global.v_body[1] << "," << state_global.omega_body[2] << std::endl;
////	std::cerr << "body acc (x, y, rot z): " << diff_global.vb_dot[0]  << ","
////			<< diff_global.vb_dot[1]  << "," << diff_global.omegab_dot[2] << std::endl;
//
	std::cerr << "i_gear: " << i_gear
<< "	a_gear: " << agear << "	diff_gear: " << agear_diff << std::endl;

	std::cerr << "Time: "   << T_global << std::endl;

    std::cerr << std::endl;

}


double dynamics::max_dynamics(double a, double b){
	if (a>=b)
		return a;
	else
		return b;

}

double dynamics::min_dynamics(double a, double b){
	if (a<=b)
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

double dynamics::CalcEngineMaxTorque(double m_engineSpeed) {

//	std::vector<std::pair<double, double> >  const torqueLookupTable;
//	= {
//    std::pair<double, double>(0.0, 0.0),
//    std::pair<double, double>(62.8318, 1660.0),
//    std::pair<double, double>(73.3038, 1880.0),
//    std::pair<double, double>(83.775, 2240.0),
//    std::pair<double, double>(94.247, 2900.0),
//    std::pair<double, double>(104.719, 3550.0),
//    std::pair<double, double>(115.191, 3550.0),
//    std::pair<double, double>(125.663, 3550.0),
//    std::pair<double, double>(136.135, 3550.0),
//    std::pair<double, double>(146.607, 3550.0),
//    std::pair<double, double>(157.079, 3470.0),
//    std::pair<double, double>(167.551, 3310.0),
//    std::pair<double, double>(178.023, 3120.0),
//    std::pair<double, double>(188.495, 2880.0),
//    std::pair<double, double>(198.967, 2660.0),
//    std::pair<double, double>(209.439, 1680.0),
//    std::pair<double, double>(219.911, 0.0)
//  };

	int size = 17;
	double torqueLookupTable[size][2] =
	{	  {0.0, 0.0},
			  {62.8318, 1660.0},
			  {73.3038, 1880.0},
			  {83.775, 2240.0},
			  {94.247, 2900.0},
			  {104.719, 3550.0},
			  {115.191, 3550.0},
			  {125.663, 3550.0},
			  {136.135, 3550.0},
			  {146.607, 3550.0},
			  {157.079, 3470.0},
			  {167.551, 3310.0},
			  {178.023, 3120.0},
			  {188.495, 2880.0},
			  {198.967, 2660.0},
			  {209.439, 1680.0},
			  {219.911, 0.0}
	};

	if (m_engineSpeed < torqueLookupTable[0][0]) {
	return 0.0;
	}

	if (m_engineSpeed >= torqueLookupTable[size- 1][0]) {
	return 0.0;
	}

	for (uint32_t i = 0; i < size - 1; i++) {
		double const x1 = torqueLookupTable[i][0];
		double const x2 = torqueLookupTable[i+1][0];

		if (m_engineSpeed >= x1 && m_engineSpeed < x2) {
 	      double const r = (m_engineSpeed - x1) / (x2 - x1);
//
    	  double const y1 = torqueLookupTable[i][1];
		  double const y2 = torqueLookupTable[i+1][1];
//
 		  double const maxTorque = y1 + r * (y2 - y1);

		  return y2;
		}
	}

	std::cerr << "Lookup failed. This should never happen." << m_engineSpeed <<  std::endl;
	return 0.0;
}





