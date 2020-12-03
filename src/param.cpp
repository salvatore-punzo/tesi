#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <stdlib.h>
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/ContactsState.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <math.h>
#include <tesi/seleziona_gamba.h>
#include "boost/thread.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/SVD"
#include <tf/tf.h>
#include "tesi/quadruped.h"
#include <std_msgs/Float64MultiArray.h>

#include "Trajectory/Trajectory.h"
#include "Trajectory/Path.h"
#include "Trajectory.cpp"
#include "Path.cpp"

#include "tesi/optimization.h"
#include "tesi/stdafx.h"
#include "ap.cpp"
#include "alglibinternal.cpp"
#include "alglibmisc.cpp"
#include "integration.cpp"
#include "interpolation.cpp"
#include "linalg.cpp"
#include "optimization.cpp"
#include "solvers.cpp"
#include "specialfunctions.cpp"


using namespace Eigen;
using namespace std;
using namespace alglib;

class ROS_SUB {

	public:
		ROS_SUB();

		void Joint_cb(sensor_msgs::JointStateConstPtr);
		void eebl_cb(gazebo_msgs::ContactsStateConstPtr);
		void eebr_cb(gazebo_msgs::ContactsStateConstPtr);
		void eefl_cb(gazebo_msgs::ContactsStateConstPtr);
		void eefr_cb(gazebo_msgs::ContactsStateConstPtr);
		void len_leg_cb(tesi::seleziona_gamba);
		void modelState_cb(gazebo_msgs::ModelStatesConstPtr);
		void Com_cb(geometry_msgs::PointStampedConstPtr);
		void VCom_cb(geometry_msgs::TwistStampedConstPtr);
		void vbody_cb(geometry_msgs::TwistStampedConstPtr);
		void run();
		void Calculate_poligono_sup();
		
		
	private:
		ros::NodeHandle _nh;
		ros::Subscriber _jointstate_sub;
		ros::Subscriber _eebl_sub;
		ros::Subscriber _eebr_sub;
		ros::Subscriber _eefl_sub;
		ros::Subscriber _eefr_sub;
		ros::Subscriber _len_leg_sub;
		ros::Subscriber _modelState_sub;
		ros::Subscriber _com_sub;
		ros::Subscriber _vcom_sub;
		ros::Subscriber _vbody_sub;


		ros::Publisher _eef_pub;
		ros::Publisher _hp_pub;
		ros::Publisher _he_pub;
		ros::Publisher _tau_pub;


		tesi::seleziona_gamba eef;
		tesi::seleziona_gamba hp;
		tesi::seleziona_gamba he;
		tesi::seleziona_gamba x_c;
		tesi::seleziona_gamba y_c;
		tesi::seleziona_gamba z_c;

		float xc_vb_rl;
		float yc_vb_rl;
		float xc_vb_ll;
		float yc_vb_ll;

		float _hblp;
		float _hbrp;
		float _hflp;
		float _hfrp;

		float _yaw_eu;
		float _pitch_eu;
		float _roll_eu;

		float _kblp;
		float _kbrp;
		float _kflp;
		float _kfrp;

		float _rblp;
		float _rbrp;
		float _rflp;
		float _rfrp;

		float _hip_j_blfr;
		float _hip_j_brfl;

		float _hip_eff_blfr;
		float _hip_eff_brfl;

		float eef_bl_z;
		float eef_br_z;
		float eef_fl_z;
		float eef_fr_z;

		Vector3d _ee_f_vb_rl;
		Vector3d _ee_f_vb_ll;

		float _len_leg_bl;
        float _len_leg_br;
        float _len_leg_fl;
        float _len_leg_fr;

		float _len_leg_blfr;
		float _len_leg_brfl;

		float x_f_base;
		float y_f_base;
		float z_f_base;

		float coo_f_base[2];

		Vector3d eef_bl_wc;
		Vector3d eef_br_wc;
		Vector3d eef_fl_wc;
		Vector3d eef_fr_wc;

		Matrix3d rot_world_virtual_base;

		float eef_vb_rlz;
		float eef_vb_llz;

		float x_com;
		float y_com;
		float z_com;

		float xdcom;
		float ydcom;
		float zdcom;

		//MatrixXd q_joints;

		Vector3d vl_floating_base;
		Vector3d va_floating_base;

		float _hblv;
		float _hbrv;
		float _hflv;
		float _hfrv;

		float _kblv;
		float _kbrv;
		float _kflv;
		float _kfrv;

		float _rblv;
		float _rbrv;
		float _rflv;
		float _rfrv;

		Matrix<double,6,1> basevel;

		QUADRUPED *doggo;

		
		
		
};



ROS_SUB::ROS_SUB() {

	_jointstate_sub = _nh.subscribe ("/dogbot/joint_states", 10, &ROS_SUB::Joint_cb, this);
	_eebl_sub = _nh.subscribe("/dogbot/back_left_contactsensor_state",10,&ROS_SUB::eebl_cb,this);
	_eebr_sub = _nh.subscribe("/dogbot/back_right_contactsensor_state",10,&ROS_SUB::eebr_cb,this);
	_eefl_sub = _nh.subscribe("/dogbot/front_left_contactsensor_state",10,&ROS_SUB::eefl_cb,this);
	_eefr_sub = _nh.subscribe("/dogbot/front_right_contactsensor_state",10,&ROS_SUB::eefr_cb,this);
	_len_leg_sub = _nh.subscribe("/data/len_leg",10,&ROS_SUB::len_leg_cb,this);
	_modelState_sub = _nh.subscribe("/gazebo/model_states", 10, &ROS_SUB::modelState_cb, this);
	_com_sub = _nh.subscribe("/dogbot/cog",10, &ROS_SUB::Com_cb,this);
	_vcom_sub = _nh.subscribe("/dogbot/v_cog",10, &ROS_SUB::VCom_cb,this);
	_vbody_sub = _nh.subscribe("/dogbot/v_body",10,&ROS_SUB::vbody_cb,this);


	_eef_pub = _nh.advertise<tesi::seleziona_gamba>("/data/eef",10);
	_hp_pub = _nh.advertise<tesi::seleziona_gamba>("/data/hip/joint",10);
	_he_pub = _nh.advertise<tesi::seleziona_gamba>("/data/hip/effort",10);
	_tau_pub = _nh.advertise<std_msgs::Float64MultiArray>("/dogbot/joint_position_controller/command",10);

	string modelFile="/home/salvatore/ros_ws/src/DogBotV4/ROS/src/dogbot_description/urdf/dogbot.urdf";

	doggo = new QUADRUPED(modelFile);
	
	
}




void ROS_SUB::Joint_cb(sensor_msgs::JointStateConstPtr js){
	
	
	
	hp.bl = js->position[1]; //back_left_pitch_joint
	hp.br= js->position[4];
	hp.fl= js->position[7];
	hp.fr= js->position[10];

	
	he.bl = js->effort[1];
	he.br = js->effort[4];
	he.fl = js->effort[7];
	he.fr = js->effort[10];

	_kblp = js->position[0]; //back_left_knee_joint
	_kbrp = js->position[3];
	_kflp = js->position[6];
	_kfrp = js->position[9];

	_rblp = js->position[2]; //back_left_roll_joint
	_rbrp = js->position[5];
	_rflp = js->position[8];
	_rfrp = js->position[11];

	_hblv = js->velocity[1];
	_hbrv = js->velocity[4];
	_hflv = js->velocity[7];
	_hfrv = js->velocity[10];

	_kblv = js->velocity[0];
	_kbrv = js->velocity[3];
	_kflv = js->velocity[6];
	_kfrv = js->velocity[9];

	_rblv = js->velocity[2];
	_rbrv = js->velocity[5];
	_rflv = js->velocity[8];
	_rfrv = js->velocity[11];

	

	

	
	
	
	
}


void ROS_SUB::Com_cb(geometry_msgs::PointStampedConstPtr pos){
	
	x_com = pos->point.x;
	y_com = pos->point.y;
	z_com = pos->point.z;
}

void ROS_SUB::VCom_cb(geometry_msgs::TwistStampedConstPtr data){
	
	xdcom = data->twist.linear.x;
	ydcom = data->twist.linear.y;
	zdcom = data->twist.linear.z;

}


void ROS_SUB::eebl_cb(gazebo_msgs::ContactsStateConstPtr eebl){
	
	float alpha = - _kblp/2; //la forza misurata dal vettore ha l'asse x nella direzione della gamba fisica. Se ruoto di questa 
	Matrix3d rot_y;//quantità porto il vettore ad avere asse z lungo la componente della gamba virtuale.
	
	rot_y << cos(alpha), 0, sin(alpha),
			0, 1, 0,
			- sin(alpha), 0, cos(alpha);
/*
	Vector3d sele_z(0,0,1);
	Vector3d n_eef_bl = rot_y * eef_bl;
	eef_bl_z = n_eef_bl.transpose() * sele_z;

		
  prova se il risultato è plausibile
	cout<<"vettore di partenza: " <<eef_bl<<endl;
	cout<<"vettore ruotato: "<<n_eef_bl<<endl;


	cout <<"componenz z:"<<comp_z<<endl;

*/
	
	//la considerazione fatta sopra alla fine non mi serve perchè io voglio il vettore nel S. di riferiento mondo
	
	Matrix3d rot_knee;
	Matrix3d rot_pitch;
	Matrix3d rot_roll;

	
	
	Vector3d eef_bl(eebl->states[0].total_wrench.force.x, eebl->states[0].total_wrench.force.y, 
	eebl->states[0].total_wrench.force.z);

	rot_knee << 1, 0, 0,
			0, cos(_kblp), -sin(_kblp),
			0, sin(_kblp), cos(_kblp);

	rot_pitch << 1, 0, 0,
			0, cos(- hp.bl), -sin(- hp.bl),
			0, sin(- hp.bl), cos(- hp.bl);
	

	
	rot_roll << cos(- _rblp), 0, sin(- _rblp),
			0, 1, 0,
			- sin(- _rblp), 0, cos(-_rblp);

	eef_bl_wc = rot_world_virtual_base * rot_roll * rot_pitch * rot_knee * eef_bl;
/*
	cout<<"forza bl nel sistema di riferimento world:"<<endl;
	cout<<eef_bl_wc <<endl;
*/
	x_c.bl = eebl->states[0].contact_positions[0].x;
	y_c.bl = eebl->states[0].contact_positions[0].y;
	z_c.bl = eebl->states[0].contact_positions[0].z;
	

}
void ROS_SUB::eebr_cb(gazebo_msgs::ContactsStateConstPtr eebr){

	Vector3d sele_z(0,0,1);
	Matrix3d rot_knee;
	Matrix3d rot_pitch;
	Matrix3d rot_roll;
	
	Vector3d eef_br( eebr->states[0].total_wrench.force.x,
	eebr->states[0].total_wrench.force.y, eebr->states[0].total_wrench.force.z);

	rot_knee << 1, 0, 0,
			0, cos(- _kbrp), -sin(- _kbrp),
			0, sin( _kbrp), cos(- _kbrp);

	rot_pitch << 1, 0, 0,
			0, cos(hp.br), -sin(hp.br),
			0, sin(hp.br), cos(hp.br);
	

	
	rot_roll << cos(- _rbrp), 0, sin(- _rbrp),
			0, 1, 0,
			- sin(- _rbrp), 0, cos(-_rbrp);

	eef_br_wc = rot_world_virtual_base * rot_roll * rot_pitch * rot_knee * eef_br;
/*
	cout<<"forza br nel sistema di riferimento world:"<<endl;
	cout<<eef_br_wc <<endl;
*/
	

	x_c.br= eebr->states[0].contact_positions[0].x;
	y_c.br= eebr->states[0].contact_positions[0].y;
	z_c.br= eebr->states[0].contact_positions[0].z;

	
}

void ROS_SUB::eefl_cb(gazebo_msgs::ContactsStateConstPtr eefl){

	Vector3d sele_z(0,0,1);
	Matrix3d rot_knee;
	Matrix3d rot_pitch;
	Matrix3d rot_roll;

	
	Vector3d eef_fl(eefl->states[0].total_wrench.force.x,
	eefl->states[0].total_wrench.force.y, eefl->states[0].total_wrench.force.z);

	rot_knee << 1, 0, 0,
			0, cos(_kflp), -sin(_kflp),
			0, sin(_kflp), cos(_kbrp);

	rot_pitch << 1, 0, 0,
			0, cos(- hp.fl), -sin(- hp.fl),
			0, sin(- hp.fl), cos(- hp.fl);
	

	
	rot_roll << cos(_rflp), 0, sin(_rflp),
			0, 1, 0,
			- sin(_rflp), 0, cos(_rflp);

	eef_fl_wc = rot_world_virtual_base * rot_roll * rot_pitch * rot_knee * eef_fl;
/*
	cout<<"forza fl nel sistema di riferimento world:"<<endl;
	cout<<eef_fl_wc <<endl;
*/	
	x_c.fl= eefl->states[0].contact_positions[0].x;
	y_c.fl= eefl->states[0].contact_positions[0].y;
	z_c.fl= eefl->states[0].contact_positions[0].z;
 
}
void ROS_SUB::eefr_cb(gazebo_msgs::ContactsStateConstPtr eefr){
	
	Vector3d sele_z(0,0,1);
	Matrix3d rot_knee;
	Matrix3d rot_pitch;
	Matrix3d rot_roll;

	Vector3d eef_fr(eefr->states[0].total_wrench.force.x,
	eefr->states[0].total_wrench.force.y, eefr->states[0].total_wrench.force.z);

	rot_knee << 1, 0, 0,
			0, cos(- _kfrp), -sin(- _kfrp),
			0, sin(- _kfrp), cos(- _kfrp);

	rot_pitch << 1, 0, 0,
			0, cos(hp.fr), -sin(hp.fr),
			0, sin(hp.fr), cos(hp.fr);
	

	
	rot_roll << cos(_rfrp), 0, sin(_rfrp),
			0, 1, 0,
			- sin(_rfrp), 0, cos(_rfrp);

	eef_fr_wc = rot_world_virtual_base * rot_roll * rot_pitch * rot_knee * eef_fr;
/*
	cout<<"forza fr nel sistema di riferimento world:"<<endl;
	cout<<eef_fr_wc <<endl;
*/
	x_c.fr= eefr->states[0].contact_positions[0].x;
	y_c.fr= eefr->states[0].contact_positions[0].y;
	z_c.fr= eefr->states[0].contact_positions[0].z;
	

}



void ROS_SUB::len_leg_cb(tesi::seleziona_gamba ll){

    _len_leg_bl = ll.bl;
    _len_leg_br = ll.br;
    _len_leg_fl = ll.fl;
    _len_leg_fr = ll.fr;



    
}

void ROS_SUB::vbody_cb(geometry_msgs::TwistStampedConstPtr vj){

	vl_floating_base << vj->twist.linear.x, vj->twist.linear.y, vj->twist.linear.z;
	va_floating_base << vj->twist.angular.x, vj->twist.angular.y, vj->twist.angular.z;
	//cout<<"velocità floating base lungo x"<<vl_floating_base[0]<<endl;
	basevel << vj->twist.linear.x, vj->twist.linear.y, vj->twist.linear.z, vj->twist.angular.x, vj->twist.angular.y, vj->twist.angular.z;

}

void ROS_SUB::Calculate_poligono_sup(){

    while(ros::ok()){
		Vector3d sele_z(0,0,1);
		float cop_x;
		float cop_y;

		//sono i parametri della gamba destra del bipede

        _len_leg_blfr = ( _len_leg_bl +_len_leg_fr )/2;
        _hip_j_blfr =( hp.bl + hp.fr )/2;
        _hip_eff_blfr = he.bl + he.fr;
        _ee_f_vb_rl = 2 * eef_bl_wc; 

		//sono i parametri della gamba sinistra del bipede

        _len_leg_brfl = ( _len_leg_br +_len_leg_fl )/2;
        _hip_j_brfl = ( hp.br + hp.fl )/2;
        _hip_eff_brfl = he.br + he.fl;
		_ee_f_vb_ll = 2 * eef_br_wc; 
	
//calcolo posizioni dei piedi del virtual biped

	xc_vb_rl = (x_c.bl + x_c.fr)/2;
	yc_vb_rl = (y_c.bl + y_c.fr)/2;

	xc_vb_ll = (x_c.br + x_c.fl)/2;
	yc_vb_ll = (y_c.br + y_c.fl)/2;
/*
	cout<<"coordinata x del piede destro del virtual biped: "<<xc_vb_rl<<endl;
	cout<<"coordinata y del piede destro del virtual biped: "<<yc_vb_rl<<endl;
	cout<<"coordinata x del piede sinistro del virtual biped: "<<xc_vb_ll<<endl;
	cout<<"coordinata y del piede sinistro del virtual biped: "<<yc_vb_ll<<endl;
	*/
//Calcolo componente z della forza 
	eef_vb_rlz =_ee_f_vb_rl.transpose() * sele_z;
	eef_vb_llz =_ee_f_vb_ll.transpose() * sele_z;

	cop_x = (eef_vb_rlz * xc_vb_rl + eef_vb_llz * xc_vb_ll)/(eef_vb_rlz+ eef_vb_llz);
	cop_y = (eef_vb_rlz * yc_vb_rl + eef_vb_llz * yc_vb_ll)/(eef_vb_rlz+ eef_vb_llz);
/*
	cout<<"copx: "<<cop_x<<endl;
	cout<<"copy: "<<cop_y<<endl;
*/


		// Calcolo vertici del poligono di supporto
		
		float m;
		float q;
		float q_newp;
		float q_newn;
		float m_pep;
		float qr;
		float qs;
		float xa,ya,xb,yb,xc,yc,xd,yd;

		m = ( yc_vb_rl - yc_vb_ll ) / ( xc_vb_rl - xc_vb_ll ); // (y2-y1)/(x2-x1)
		q = - xc_vb_ll * m + yc_vb_ll;//-x1*m +y1

		q_newp = q + 0.01;
		q_newn = q - 0.01;
		m_pep = -1/m; //m perpendicolare
		qr = yc_vb_ll - m_pep * xc_vb_ll;
		qs = yc_vb_rl - m_pep * xc_vb_rl;

		xa=(qr-q_newp)/((pow(m,2)+1)/m);
		ya=-(qr - q_newp)/(pow(m,2)+1)+ qr;

		xb=(qs-q_newp)/((pow(m,2)+1)/m);
		yb=-(qs - q_newp)/(pow(m,2)+1)+ qs;

		xc=(qs-q_newn)/((pow(m,2)+1)/m);
		yc=-(qs - q_newn)/(pow(m,2)+1)+ qs;

		xd=(qr-q_newn)/((pow(m,2)+1)/m);
		yd=-(qr - q_newn)/(pow(m,2)+1)+ qr;
/* test
		cout<<"A: "<<xa<<","<<ya<<endl;
		cout<<"B: "<<xb<<","<<yb<<endl;
		cout<<"C: "<<xc<<","<<yc<<endl;
		cout<<"D: "<<xd<<","<<yd<<endl;

		cout<<"copx: "<<cop_x<<endl;
		cout<<"copy: "<<cop_y<<endl;

		*/
	

    }

}

void ROS_SUB::modelState_cb(gazebo_msgs::ModelStatesConstPtr pt){
	
	double yaw_eu, pitch_eu, roll_eu;

	_yaw_eu = (float) yaw_eu;
	_pitch_eu = (float) pitch_eu;
	_roll_eu = (float) roll_eu;

	_hfrp = hp.fr;
	_hflp = hp.fl;
	_hbrp = hp.br;
	_hblp = hp.bl;
	
	/*			prova
	float a;
	a= pt->pose[1].orientation.x;  //pose è un vettore di cardinalità 3 che mi permette di
	// accedere alla posizione e all'orientamento diel ground_plane (0), del dogbot(2), del plannar_mover(1)
	cout<<a<<endl;
	*/


	tf::Quaternion q(pt->pose[2].orientation.x, pt->pose[2].orientation.y, pt->pose[2].orientation.z, pt->pose[2].orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	tf::Matrix3x3(q).getEulerYPR(yaw_eu,pitch_eu,roll_eu);


	rot_world_virtual_base << cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll),
							sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll),
							-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll);

	x_f_base = pt->pose[2].position.x;
	y_f_base = pt->pose[2].position.y;
	z_f_base = pt->pose[2].position.z;
/*
	coo_f_base[0] = pt->pose[2].position.x;
	coo_f_base[1] = pt->pose[2].position.y;
	coo_f_base[2] = pt->pose[2].position.z;
*/
/*  test
	cout<<"coordinate floating base:"<<endl;
	cout<<"x: "<<coo_f_base[0]<<endl;
	cout<<"y: "<<coo_f_base[1]<<endl;
	cout<<"z: "<<coo_f_base[2]<<endl;
	cout<<"angoli di rotazioni terna base-floating base"<<endl;
	cout<<"roll: "<<roll<<endl;
	cout<<"pitch: "<<pitch<<endl;
	cout<<"yaw: "<<yaw<<endl;
	*/

	//VectorXd q_joints( (double) x_f_base, (double) y_f_base,(double) z_f_base, (double) _roll_eu, (double) _pitch_eu, (double) _yaw_eu, (double) _rblp, (double) _hblp, (double) _kblp , (double) _rbrp, (double) _hbrp, (double) _kbrp, (double) _rflp, (double) _hflp, (double) _kflp, (double) _rfrp, (double) _hfrp, (double) _kfrp);
	
	VectorXd q_joints(12,1);
	VectorXd dq_joints(12,1);
	ArrayXd vectorZero = ArrayXd::Zero(18);
	Matrix<double,6,1> q_joints_base;
	q_joints_base<<x_f_base, y_f_base, z_f_base, _roll_eu, _pitch_eu, _yaw_eu;
	//MatrixXd q_joints_j(12,1);
	MatrixXd zeros = MatrixXd::Zero(12,6);
	MatrixXd eye = MatrixXd::Identity(12,12);

	Matrix<double,3,3> I = Matrix<double,3,3>::Identity(3,3);
	Matrix<double,3,3> zero = Matrix<double,3,3>::Zero(3,3);

	Matrix<double,12,18> S;
	Matrix<double,18,12> S_T;
	Matrix4d world_H_base;
	Matrix<double,24,12> B;

	B<< I,zero,zero,zero,
		zero,zero,zero,zero,
		zero,I,zero,zero,
		zero,zero,zero,zero,
		zero,zero,I,zero,
		zero,zero,zero,zero,
		zero,zero,zero,I,
		zero,zero,zero,zero;


	// la matrice di rotazione  quella ricavata con gli angoli roll, pitch ,yaw
	world_H_base << cos(yaw)*cos(pitch), cos(yaw)*sin(pitch)*sin(roll)-sin(yaw)*cos(roll), cos(yaw)*sin(pitch)*cos(roll) + sin(yaw)*sin(roll), x_f_base,
					sin(yaw)*cos(pitch), sin(yaw)*sin(pitch)*sin(roll)+cos(yaw)*cos(roll), sin(yaw)*sin(pitch)*cos(roll)-cos(yaw)*sin(roll), y_f_base,
					-sin(pitch), cos(pitch)*sin(roll), cos(pitch)*cos(roll), z_f_base,
					0, 0, 0, 1;



	
	S << zeros, eye;
	S_T = S.transpose();
	

	/* verifica delle matrici
	cout<<"Matrice S:"<<endl;
	cout<<S<<endl;
	
	cout<<"Matrice S transposta:"<<endl;
	cout<<S_T<<endl;
	*/


	q_joints << _rfrp, _rflp, _rblp, _rbrp, _hbrp,  _kbrp, _hblp,  _kblp , _hflp, _kflp, _hfrp,  _kfrp;
	dq_joints << _rfrv, _rflv, _rblv, _rbrv, _hbrv,  _kbrv, _hblv,  _kblv , _hflv, _kflv, _hfrv,  _kfrv;
	Vector3d gravity1(0,0, -9.81);
	
	doggo->update(world_H_base, q_joints, dq_joints, basevel, gravity1);
	VectorXd b=doggo->getBiasMatrix();
	Matrix<double,18,18> M=doggo->getMassMatrix();
	Matrix<double,24,18> Jc=doggo->getJacobian();
	Matrix<double,24,1> Jcdqd=doggo->getBiasAcc();
	Matrix<double,18,18> T=doggo->getTransMatrix();
	Matrix<double,18,18> T_dot=doggo->getTdotMatrix();
	Matrix<double,18,12> Jc_T_B;
	Matrix<double,12,18> B_T_Jc;
	Matrix<double, 18,1> q_joints_total;
	Matrix<double, 18,1> dq_joints_total;
	Matrix<double,4,12> Sn;
	Matrix<double,3,18> Jt1;
	Matrix<double,3,18> Jt1_T;
	Matrix<double,3,1> Jt1_Tdot_dq;
	
	int CT[37];
	Vector3d e;
	Vector3d e_dot;
	double kp=4;
	double kd=2*sqrt(kp);

	//-------------------------- Inizio traiettoria --------------------------
	double com_zdes = 0.4;
	double dist = com_zdes-z_com;
	double step = dist/100;
 
	list<VectorXd> waypoints;
	VectorXd waypoint(3);
	for (int i = 0; i<100; i++){
		
		waypoint << x_com, y_com, z_com + step*i;
		waypoints.push_back(waypoint);
	}
	VectorXd maxAcceleration(3);
	maxAcceleration << 1.0, 1.0, 1.0;
	VectorXd maxVelocity(3);
	maxVelocity << 1.0, 1.0, 1.0;

	Trajectory trajectory(Path(waypoints, 0.1), maxVelocity, maxAcceleration);
	trajectory.outputPhasePlaneTrajectory();

	double duration = trajectory.getDuration();

	

	//Jacobian task 1
	

	Jt1<<Matrix<double,1,18>::Zero(),
		Matrix<double,1,18>::Zero(),
		0,0,1,Matrix<double,1,15>::Zero();

	Jt1_T<<Jt1 * T;

	Jt1_Tdot_dq<<Jt1 * T_dot * dq_joints_total;

	//Sn è la matrice che seleziona le componenti sull'asse z delle forze di contatto
	Sn<<0,0,1,Matrix<double,1,9>::Zero(),
		Matrix<double,1,5>::Zero(),1,Matrix<double,1,6>::Zero(),
		Matrix<double,1,8>::Zero(),1,Matrix<double,1,3>::Zero(),
		Matrix<double,1,11>::Zero(),1;

	q_joints_total<<q_joints_base, q_joints;
	dq_joints_total<<vl_floating_base,va_floating_base,dq_joints;
	

	Jc_T_B = Jc.transpose() * B;
	B_T_Jc = B.transpose() * Jc;



	//imposto il problema quadratico
	Matrix<double,43,43> aa;
	aa<<Matrix<double,43,43>::Zero();
	aa(42,42)=1;
	real_2d_array a;
	a.setlength(43,43);
	a.setcontent(43,43,&aa(0,0));
	
  
    real_1d_array s;
	s.setlength(43);
	for(int i = 0; i< 43; i++){
		
		s(0)=1;
	} 
	


	//vincoli del problema quadratico 


	real_2d_array c; 
	
	
	Matrix<double,37,44> A;

for(double t = 0.0; t < duration; t += 0.1) {
		e<<0,0,z_com-trajectory.getPosition(t)[2];
		e_dot<<0,0,trajectory.getVelocity(t)[2];
	
	A<<M,Jc_T_B,S_T,Matrix<double,18,1>::Zero(),-b,
		B_T_Jc, Matrix<double,12,25>::Zero(),B.transpose()*Jcdqd,
		Matrix<double,4,18>::Zero(), Sn, Matrix<double,4,13>::Zero(),Matrix<double,4,1>::Zero(),
		Jt1_T, Matrix<double,3,24>::Zero(),Matrix<double,3,1>::Ones(), -Jt1_Tdot_dq -kd*e_dot - kp * e;
	
	//cout<<"A:"<<endl;
	//cout<<A<<endl;
	c.setlength(37,44);
	c.setcontent(37,44, &A(0,0));

	
	

	//parametro che imposta la tipologia del vincolo 
	integer_1d_array ct;
	ct.setlength(37);
	for(int i = 0; i <30; i++){
		ct(i) = 0;
	}
	for(int i = 30; i<37; i++){
		ct(i) = 1;
	}

	//box constrain
	real_1d_array bndl;
	bndl.setlength(43);
	real_1d_array bndu;
	bndu.setlength(43);
	for(int i = 0; i<30; i++){
		bndl(i)=-20;// fp_neginf;//-INFINITY
		bndu(i)=20;// fp_posinf;//+INFINITY
	}

	for(int i=30; i<42; i++){
		bndl(i)= -60;
		bndu(i)= 60;
	}

	bndl(42)= fp_neginf;//-Infinity
	bndu(42)= fp_posinf;//+Infinity

	
	real_1d_array x;
	minqpstate state;
    minqpreport rep;

	 // NOTE: for convex problems you may try using minqpsetscaleautodiag()
    //       which automatically determines variable scales.
	minqpsetscale(state, s);
	// create solver, set quadratic/linear terms
	
    minqpcreate(43, state);
    minqpsetquadraticterm(state, a);
	//minqpsetlinearterm(state, l); non mi servono perchè di default sono già impostati a zero
    minqpsetlc(state, c, ct);
	minqpsetbc(state, bndl, bndu);
	
	minqpsetalgobleic(state, 0.0, 0.0, 0.0, 0);
    minqpoptimize(state);

	minqpresults(state, x, rep);

    printf("%s\n", x.tostring(1).c_str());
	
	vector<double> tau = { x(30),x(40),x(41),x(31),x(38),x(39),x(33),x(34),x(35),x(32),x(36),x(37)};
	
	std_msgs::Float64MultiArray msg;

	// set up dimensions
	msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
	msg.layout.dim[0].size = tau.size();
	msg.layout.dim[0].stride = 1;
	msg.layout.dim[0].label = "x"; // or whatever name you typically use to index vec1

	// copy in the data
	msg.data.clear();
	msg.data.insert(msg.data.end(), tau.begin(), tau.end());
	//tau<<x(30),x(40),x(41),x(31),x(38),x(39),x(33),x(34),x(35),x(32),x(36),x(37);
	_tau_pub.publish(msg);
	}
	//printf("%d\n", int(rep.terminationtype)); // se positiva la soluzione esiste
	//cout<<"x:"<<x(0)<<endl;





}


void ROS_SUB::run(){
    boost::thread ( &ROS_SUB::Calculate_poligono_sup, this );
}



int main(int argc, char **argv){
	
	ros::init(argc, argv, "get_param");
	
	ROS_SUB rs;
	rs.run();
	
	
	ros::spin();
	
	return 0;

}
