#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <stdlib.h>
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/ContactsState.h"
#include "gazebo_msgs/ModelStates.h"
#include "geometry_msgs/Vector3.h"
#include <math.h>
#include <tesi/seleziona_gamba.h>
#include "boost/thread.hpp"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/SVD"
#include <tf/tf.h>

using namespace Eigen;
using namespace std;

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

		ros::Publisher _eef_pub;
		ros::Publisher _hp_pub;
		ros::Publisher _he_pub;

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

		float _kblp;
		float _kbrp;
		float _kflp;
		float _kfrp;

		float _hip_j_blfr;
		float _hip_j_brfl;

		float _hip_eff_blfr;
		float _hip_eff_brfl;

		float eef_bl_z;
		float eef_br_z;
		float eef_fl_z;
		float eef_fr_z;

		float _ee_f_blfr;
		float _ee_f_brfl;

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




		
	
		
		
};



ROS_SUB::ROS_SUB() {

	_jointstate_sub = _nh.subscribe ("/dogbot/joint_states", 10, &ROS_SUB::Joint_cb, this);
	_eebl_sub = _nh.subscribe("/dogbot/back_left_contactsensor_state",10,&ROS_SUB::eebl_cb,this);
	_eebr_sub = _nh.subscribe("/dogbot/back_right_contactsensor_state",10,&ROS_SUB::eebr_cb,this);
	_eefl_sub = _nh.subscribe("/dogbot/front_left_contactsensor_state",10,&ROS_SUB::eefl_cb,this);
	_eefr_sub = _nh.subscribe("/dogbot/front_right_contactsensor_state",10,&ROS_SUB::eefr_cb,this);
	_len_leg_sub = _nh.subscribe("/data/len_leg",10,&ROS_SUB::len_leg_cb,this);
	_modelState_sub = _nh.subscribe("/gazebo/model_states", 10, &ROS_SUB::modelState_cb, this);

	_eef_pub = _nh.advertise<tesi::seleziona_gamba>("/data/eef",10);
	_hp_pub = _nh.advertise<tesi::seleziona_gamba>("/data/hip/joint",10);
	_he_pub = _nh.advertise<tesi::seleziona_gamba>("/data/hip/effort",10);
	
}




void ROS_SUB::Joint_cb(sensor_msgs::JointStateConstPtr js){
	
	//mi servono le 4 posizione dei giunti di hip e le 4 coppie applicate a questi
	
	hp.bl = js->position[1]; //in questo modo prendo il secondo elemento del vettore position nello specifico:back_left_pitch_joint
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


	

	
	
	
	
}

void ROS_SUB::eebl_cb(gazebo_msgs::ContactsStateConstPtr eebl){
	
	float alpha = - _kblp/2;

	Vector3d eef_bl(eebl->states[0].total_wrench.force.x, eebl->states[0].total_wrench.force.y, 
	eebl->states[0].total_wrench.force.z);

	Matrix3d rot_y;
	rot_y << cos(alpha), 0, sin(alpha),
			0, 1, 0,
			- sin(alpha), 0, cos(alpha);

	Vector3d sele_z(0,0,1);
	Vector3d n_eef_bl = rot_y * eef_bl;
	eef_bl_z = n_eef_bl.transpose() * sele_z;
/*	
 // prova se il risultato è plausibile
	cout<<"vettore di partenza: " <<eef_bl<<endl;
	cout<<"vettore ruotato: "<<n_eef_bl<<endl;


	cout <<"componenz z:"<<comp_z<<endl;

*/
	x_c.bl = eebl->states[0].contact_positions[0].x;
	y_c.bl = eebl->states[0].contact_positions[0].y;
	z_c.bl = eebl->states[0].contact_positions[0].z;
	

}
void ROS_SUB::eebr_cb(gazebo_msgs::ContactsStateConstPtr eebr){

		float alpha = - _kbrp/2;
	
	Vector3d eef_br( eebr->states[0].total_wrench.force.x,
	eebr->states[0].total_wrench.force.y, eebr->states[0].total_wrench.force.z);

	Matrix3d rot_y;
	rot_y << cos(alpha), 0, sin(alpha),
			0, 1, 0,
			- sin(alpha), 0, cos(alpha);


	Vector3d sele_z(0,0,1);

	Vector3d n_eef_br = rot_y * eef_br;
	eef_br_z = n_eef_br.transpose() * sele_z;

	x_c.br= eebr->states[0].contact_positions[0].x;
	y_c.br= eebr->states[0].contact_positions[0].y;
	z_c.br= eebr->states[0].contact_positions[0].z;

	//cout<<eebr->states[0];
}

void ROS_SUB::eefl_cb(gazebo_msgs::ContactsStateConstPtr eefl){

			float alpha = - _kflp/2;

	
	Vector3d eef_fl(eefl->states[0].total_wrench.force.x,
	eefl->states[0].total_wrench.force.y, eefl->states[0].total_wrench.force.z);

	Matrix3d rot_y;
	rot_y << cos(alpha), 0, sin(alpha),
			0, 1, 0,
			- sin(alpha), 0, cos(alpha);


	Vector3d sele_z(0,0,1);
	Vector3d n_eef_fl = rot_y * eef_fl;
	eef_fl_z = n_eef_fl.transpose() * sele_z;
	
	x_c.fl= eefl->states[0].contact_positions[0].x;
	y_c.fl= eefl->states[0].contact_positions[0].y;
	z_c.fl= eefl->states[0].contact_positions[0].z;
 
}
void ROS_SUB::eefr_cb(gazebo_msgs::ContactsStateConstPtr eefr){
	
	float alpha = - _kfrp/2;

	Vector3d eef_fr(eefr->states[0].total_wrench.force.x,
	eefr->states[0].total_wrench.force.y, eefr->states[0].total_wrench.force.z);

	Matrix3d rot_y;
	rot_y << cos(alpha), 0, sin(alpha),
			0, 1, 0,
			- sin(alpha), 0, cos(alpha);


	Vector3d sele_z(0,0,1);
	Vector3d n_eef_fr = rot_y * eef_fr;
	eef_fr_z = n_eef_fr.transpose() * sele_z;

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

 void ROS_SUB::Calculate_poligono_sup(){

    while(ros::ok()){

		//sono i parametri della gamba destra del bipede

        _len_leg_blfr = ( _len_leg_bl +_len_leg_fr )/2;
        _hip_j_blfr =( hp.bl + hp.fr )/2;
        _hip_eff_blfr = he.bl + he.fr;
        _ee_f_blfr = 2 * eef_bl_z; 

		//sono i parametri della gamba sinistra del bipede

        _len_leg_brfl = ( _len_leg_br +_len_leg_fl )/2;
        _hip_j_brfl = ( hp.br + hp.fl )/2;
        _hip_eff_brfl = he.br + he.fl;
		_ee_f_brfl = 2 * eef_br_z; 
	/*
		//float y;
		//float x;
		
		float m;
		float q;

		m = ( y_c.fr - y_c.bl ) / ( x_c.fr-x_c.bl ); // (y2-y1)/(x2-x1)
		q = - x_c.bl * m + y_c.bl;//-x1*m +y1


		//y=m*x+q;
	*/
//calcolo posizioni dei piedi del virtual biped

	xc_vb_rl = (x_c.bl + x_c.fr)/2;
	yc_vb_rl = (y_c.bl + y_c.fr)/2;

	xc_vb_ll = (x_c.br + x_c.fl)/2;
	yc_vb_ll = (y_c.br + y_c.fl)/2;
	
    }

}

void ROS_SUB::modelState_cb(gazebo_msgs::ModelStatesConstPtr pt){
	/*			prova
	float a;
	a= pt->pose[1].orientation.x;  //pose è un vettore di cardinalità 3 che mi permette di
	// accedere alla posizione e all'orientamento diel ground_plane (0), del dogbot(1), del plannar_mover(2)
	cout<<a<<endl;
	*/

	tf::Quaternion q(pt->pose[1].orientation.x, pt->pose[1].orientation.y, pt->pose[1].orientation.z, pt->pose[1].orientation.w);
	double roll, pitch, yaw;
	tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

	x_f_base = pt->pose[1].position.x;
	y_f_base = pt->pose[1].position.y;
	z_f_base = pt->pose[1].position.z;

	coo_f_base[0] = pt->pose[1].position.x;
	coo_f_base[1] = pt->pose[1].position.y;
	coo_f_base[2] = pt->pose[1].position.z;

	cout<<"coordinate floating base:"<<endl;
	cout<<"x: "<<coo_f_base[0]<<endl;
	cout<<"y: "<<coo_f_base[1]<<endl;
	cout<<"z: "<<coo_f_base[2]<<endl;
	cout<<"angoli di rotazioni terna base-floating base"<<endl;
	cout<<"roll: "<<roll<<endl;
	cout<<"pitch: "<<pitch<<endl;
	cout<<"yaw: "<<yaw;

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
