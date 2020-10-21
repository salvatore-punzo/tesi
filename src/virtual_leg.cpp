#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <stdlib.h>
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/ContactsState.h"
#include "geometry_msgs/Vector3.h"
#include <math.h>
#include <tesi/seleziona_gamba.h>
#include <tesi/parametri_virtual_leg.h> 
#include "boost/thread.hpp"

using namespace std;

class Virtual_Leg{

    public:
        Virtual_Leg();
        void Calculate_VL_cb();
        void hip_joint_cb(tesi::seleziona_gamba);
        void len_leg_cb(tesi::seleziona_gamba);
        void hip_effort_cb(tesi::seleziona_gamba);
        void ee_force_cb(tesi::seleziona_gamba);
        void run();
        

    private:
        ros::NodeHandle _nh;

		ros::Subscriber _hip_joint_sub;
        ros::Subscriber _len_leg_sub;
        ros::Subscriber _hip_effort_sub;
        ros::Subscriber _ee_force_sub;

        ros::Publisher _parametri_vlr_pub;
        ros::Publisher _parametri_vll_pub;



        float _hip_eff_bl;
        float _hip_eff_br;
        float _hip_eff_fl;
        float _hip_eff_fr;

        float _hip_j_bl;
        float _hip_j_br;
        float _hip_j_fl;
        float _hip_j_fr;

        float _len_leg_bl;
        float _len_leg_br;
        float _len_leg_fl;
        float _len_leg_fr;

        float _ee_f_bl;
        float _ee_f_br;
        float _ee_f_fl;
        float _ee_f_fr;

        float _len_leg_blfr;
        float _hip_j_blfr;
        float _hip_eff_blfr;
        float _ee_f_blfr;

        float _len_leg_brfl;
        float _hip_j_brfl;
        float _hip_eff_brfl;
        float _ee_f_brfl;


        tesi::parametri_virtual_leg rl;
        tesi::parametri_virtual_leg ll;




};

Virtual_Leg::Virtual_Leg(){

    _hip_joint_sub = _nh.subscribe("/data/hip/joint",30,&Virtual_Leg::hip_joint_cb,this);
    _len_leg_sub = _nh.subscribe("/data/len_leg",30,&Virtual_Leg::len_leg_cb,this);
    _hip_effort_sub =_nh.subscribe("/data/hip/effort",30,&Virtual_Leg::hip_effort_cb,this);
    _ee_force_sub =_nh.subscribe("/data/eef",30,&Virtual_Leg::ee_force_cb,this);
    
    _parametri_vlr_pub = _nh.advertise<tesi::parametri_virtual_leg>("/data/virtual_leg_r",30);
    _parametri_vll_pub = _nh.advertise<tesi::parametri_virtual_leg>("/data/virtual_leg_l",30);
}

void Virtual_Leg::hip_effort_cb(tesi::seleziona_gamba he){

    _hip_eff_bl = he.bl;
    _hip_eff_br = he.br;
    _hip_eff_fl = he.fl;
    _hip_eff_fr = he.fr;

}

void Virtual_Leg::hip_joint_cb(tesi::seleziona_gamba hj){

    _hip_j_bl = hj.bl;
    _hip_j_br = hj.br;
    _hip_j_fl = hj.fl;
    _hip_j_fr = hj.fr;


}

void Virtual_Leg::len_leg_cb(tesi::seleziona_gamba ll){

    _len_leg_bl = ll.bl;
    _len_leg_br = ll.br;
    _len_leg_fl = ll.fl;
    _len_leg_fr = ll.fr;



    
}

void Virtual_Leg::ee_force_cb(tesi::seleziona_gamba eef){

    _ee_f_bl = eef.bl;
    _ee_f_br = eef.br;
    _ee_f_fl = eef.fl;
    _ee_f_fr = eef.fr;
    
}



void Virtual_Leg::Calculate_VL_cb(){

    while(ros::ok()){
        //sono i parametri della gamba destra del bipede
        _len_leg_blfr = ( _len_leg_bl +_len_leg_fr )/2;
        _hip_j_blfr =( _hip_j_bl + _hip_j_fr )/2;
        _hip_eff_blfr = _hip_eff_bl + _hip_eff_fr;
        _ee_f_blfr = 2 * _ee_f_bl; 

        //sono i parametri della gamba sinistra del bipede
        _len_leg_brfl = ( _len_leg_br +_len_leg_fl )/2;
        _hip_j_brfl = ( _hip_j_br + _hip_j_fl )/2;
        
        _hip_eff_brfl = _hip_eff_br + _hip_eff_fl;

        _ee_f_brfl = 2 * _ee_f_br; 

        cout<<"lunghezza leg: "<<_len_leg_blfr<<endl;
        cout<<"hip joint: "<<_hip_j_blfr<<endl;
/*  vedi paper per capire bene come assegnare i parametri rl = right leg e ll = left leg
        rl.length = _len_leg_blfr;
        rl.teta = _hip_j_blfr;
        rl.tau =
        rl.force =

        ll.length = _len_leg_brfl;
        ll.teta = _hip_j_brfl;
        ll.tau =
        ll.force =

        _parametri_vrl_pub.publish(rl);
        _parametri_vll_pub.publish(ll);
        */
    }

}

void Virtual_Leg::run(){
    boost::thread ( &Virtual_Leg::Calculate_VL_cb, this );
    //ros::spin();
}



int main(int argc, char **argv){
	
	ros::init(argc, argv, "virtual_leg");
	
	Virtual_Leg rs;
	
    rs.run();
	
	ros::spin();
	
	return 0;

}