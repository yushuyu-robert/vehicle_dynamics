/*
 * dynamics_vehicle.h
 *
 *  Created on: Jan 22, 2018
 *      Author: yushu
 */

#ifndef DYNAMICS_VEHICLE_H_
#define DYNAMICS_VEHICLE_H_




//other header files


#include "ros/ros.h"
#include "std_msgs/String.h"


//#include "sensor_msgs/Imu.h"





class dynamics{
public:
	dynamics();
private:


//from 22nd, Jan., 2017,  the dynamics of the vehicles


	//the parameters of the vehicle:
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

      int i_final;
      int i_gear;
      double eta_tr;
      double eta_fd;
      int r_gear;

      //upper and lower bounds of acceleration limits,
      double a_xupper;
      double a_xlower;


      //input:
      double A_ped;   //pedal
      double B_ped;  //brake
      double steering_angle;



	//the functions and other variables:


	   void diff_equation(void);

       double max_dynamics(double a, double b);
       double abs_dynamics(double a);
       double f_omegae(double omega_e);



	   //the velocity of the vehicle, expressed in the body frame of the vehicle
	   double v_body[3];
	   double omega_body[3];  //angular velocity, body frame

	   //the velocity of the vehicle, expressed in the wheel frame
	   double vb_wheel[3][2];

	   double omega_w[2];  //the angular velocity of the wheel

	   double delta[2];  //the steering angle



	   //slip
	   double sx[2];
	   double sy[2];
	   double sxy[2];

	   double f_sxy[2];
	   double Fxy[2];
	   double Fz[2];
	   double Fw[3][2];  //force expressed in wheel frame
	   double Fv[3][2];  //force expressed in body frame

	   double Fx; //total force
	   double Fy;
	   double F_d[2];
	   double F_g[2];


	   double omega_wheel_dot[2];
	   double vb_dot[3];  //dot of velocity of body
	   double omegab_dot[3];  //dot of angular velocity of body

	 //  omega_wheel_dot = (T_prop - T_brk - Fw[0] * rw - T_roll);
	   double T_prop[2];
	   double T_brk[2];
	   double T_roll[2];


	   //the fraction by which the engine torque is reduecd
	   double Efactor;

	   double omega_e_dot;

	   //braking torque:
	   double T_b[2];  //front and rear wheel


































    //send command in acc mode
    void send_acc_ctrl(void);

    //send command in velocity mode:
    void send_velo_control(void);






    ros::NodeHandle n;
    ros::NodeHandle pnh_;

    ros::Publisher pub;
    ros::Publisher pub2;

    ros::Publisher pub3;

    ros::Publisher llcmd_pub_acc;

    ros::Publisher llcmd_pub_vel;

    ros::Publisher ext_state;
    ros::Publisher position_gps;  //publish the position from GPS

    ros::Publisher pose_ssdk; //publish pose

    ros::Subscriber pose_sub_;  //subscribe the current position of the UAV, from SLAM module
    ros::Subscriber odometry_sub_;  //the topic name is still under discussion, from the SLAM module


    ros::Subscriber rcdata_sub_;

    ros::Subscriber gps_custom_sub_;
    ros::Subscriber imu_custom_sub_;

    ros::Subscriber sub;

    ros::Subscriber flag_cmd_sub;
    ros::Subscriber cmdfromgene_sub;

    ros::Subscriber debugdata_sub;  //Subscribe the debug data, from the HLP

    ros::Timer timer_pubstate;





























    //determine if the RC transmitter sends the position commands
    int flag_rc_cmd;

    //out door or indoor,
    //1: outdoor, GPS provides the position information
    //2: indoor, SLAM module provides the position information
    int flag_pose_source;

  //global position commands, used to calculate the position commands from RC transmitter

    int64_t time;
    double time_body, time_doby_last;


    /// gain from AutoPilot values to 1/1000 degree for the input from the pitch and roll "stick"
    /**
     * This value should be equal to the one that can be found when reading the controller parameters of the LLP by the AscTec AutoPilot software.
     * It is only relevant, when this interface is used to send roll/pitch (or x/y velocity when in GPS mode) commands to the LLP.
     */
    int k_stick_;

    /// gain from AutoPilot values to 1/1000 degree for the input from the yaw "stick"
    /**
     * This value should be equal to the one that can be found when reading the controller parameters of the LLP by the AscTec AutoPilot software.
     * It is only relevant, when this interface is used to send yaw commands to the LLP.
     */
    int k_stick_yaw_;


    //added on Feb., 2016, position control
    void translate_innerloop_controller(void);
    void translate_outerloop_controller(void);
    void compute_F_nom(void);
    void compute_V_nom(void);
    void compute_gamma_nom(void);
    double norm_vector(double *p);
    void cross_vector(double *a1, double *a2, double *result);
    double dot_product(double *a1, double *a2);
    void compute_omega_nom(void);
    void rotate_outerloop_controller(void);
    double error_yaw(double yaw_com, double yaw_sens);

	////////////////below, the outer loop of the translational controller////////// added on 27th, September, for 6DOF
	double Px_nom;
	double Py_nom;
	double Pz_nom;
	double P_nom_filter_m2[3];
	double P_nom_filter_m1[3];
	double xi_filter_out_trans[3];  //????
	double omega_n_filter_out_trans[3];  //????
	double a_1_out_trans[3];
	double a_2_out_trans[3];
	double P_nom[3];
	double V_nom[3];
	double V_com[3];
	double V_ctrl[3];
	double P_err[3];
	double P_err_int[3];
	double P_sen[3];
	////////////////above, the outer loop of the translational controller///////// added on 27th, September, for 6DOF

	////////////////below, the inner loop of the translational controller//////// added on 27th, September, for 6DOF
	double Vx_nom;
	double Vy_nom;
	double Vz_nom;
	double V_nom_filter_m2[3];
	double V_nom_filter_m1[3];
	double xi_filter_in_trans[3];  //????
	double omega_n_filter_in_trans[3];  //????
	double a_1_in_trans[3];
	double a_2_in_trans[3];
	double F_nom[3];
	double F_com[3];
	double F_ctrl[3];
	double V_err[3];
	double V_err_int[3];
	double V_sen[3];
	double V_sen_b[3];
	////////////////above, the inner loop of the translational controller/////// added on 27th, September, for 6DOF

	////////the parameters in the feedback control
	double ksi_trans_out_x;
	double ksi_trans_out_y;
	double ksi_trans_out_z;
	double omega_trans_out_x;
	double omega_trans_out_y;
	double omega_trans_out_z;

	double ksi_trans_in_x;
	double ksi_trans_in_y;
	double ksi_trans_in_z;
	double omega_trans_in_x;
	double omega_trans_in_y;
	double omega_trans_in_z;

	double ksi_roll_out;
	double ksi_pitch_out;
	double ksi_yaw_out;
	double omega_roll_out;
	double omega_pitch_out;
	double omega_yaw_out;

	double ksi_roll_in;
	double ksi_pitch_in;
	double ksi_yaw_in;
	double omega_roll_in;
	double omega_pitch_in;
	double omega_yaw_in;

	//sampling time:
	double T_sampling;
	uint16_t time_scale_position;  //the scale of the sampling time of the position time compared to attitude loop

	double m;
	double g_;
	double G;
	double G_6dof_init;

	/////below, the variables used to compute the nominal and commanded Euler angles// added on 27th, September, for 6DOF
	double gamma_ctrl[3];
	double gamma_err[3];
	double gamma_com[3];
	double gamma_nom[3];
	double gamma_sen[3];
	double f_z_com;
	double yaw_6DOF_init;
	double Rzb_sens[3];
	double Fcom_exg[3];
	double z_b[3];
	double a_1[3];
	double y_b[3];
	double x_b[3];
	/////above, the variables used to compute the nominal and commanded Euler angles/// added on 27th, September, for 6DOF


	//the variables used in the attitude control
	double omega_nom[3];
	double omega_com[3];  /*?????????*/
	double omega_err[3];//={0,0,0};
	double omega_ctrl[3];

	double gamma_err_int[3];//={0,0,0};
	double omega_err_int[3];//={0,0,0};

	double phi_nom;//=0;		//gamma_nom[0];
	double theta_nom;//=0;
	double psi_nom;//=0;

	double p_nom;//=0;			//omega_nom[0]
	double q_nom;//=0;
	double r_nom;//=0;
	//above, the variables used in the attitude control

	int flag_control;


	//////////the parameters in the pseudo inverse dynamics of attitude control loop
	double gamma_nom_filter_m1[3];
	double omega_nom_filter_m1[3];
	double gamma_nom_filter_m2[3];
	double omega_nom_filter_m2[3];
	////////???????????/////??/////////////////
	double xi_filter_out[3];  //?????????
	double omega_n_filter_out[3];  //?????????
	double a_1_out[3];   //?????????
	double a_2_out[3];  //?????????
	double xi_filter_in[3];  //?????????
	double omega_n_filter_in[3];  //?????????
	double a_1_in[3];   //?????????
	double a_2_in[3];  //?????????
	//above, the parameters in the pseudo inverse dynamics of attitude control loop

	//2017 May, added for the initialization of SLAM
	int slam_int; //flag determine if it is the initial time of SLAM
	int slam_int_instant; //the intial instant
	double yaw_ini_slam; //the yaw angle at the initial time, when SLAM is available


	//used in simulation:
	double P_sim[3];
	double V_sim[3];
	double gamma_sim[3];
	double m_sim;
	double g_sim; //acc due to gravity
	void translation_eom(void);
	int flag_sim;
	int flag_ini_control; //used in the initialization of control, 1, 2, 3, 4, 5, initial value is 1
	int flag_mode_control; //used in the control mode selection , 1, 2, 3, initial value is 1,

	void timerCallback(const ros::TimerEvent& event);

};




















#endif /* DYNAMICS_VEHICLE_H_ */
