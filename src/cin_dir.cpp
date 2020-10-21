#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <iostream>
#include <stdlib.h>
#include "sensor_msgs/JointState.h"
#include "gazebo_msgs/ContactsState.h"
#include "geometry_msgs/Vector3.h"
#include <math.h>
#include <tesi/seleziona_gamba.h>

using namespace std;

class CIN_DIR{
    public:
        CIN_DIR();
        void calculate_leg_length_cb(sensor_msgs::JointStateConstPtr);

    private:
        ros::NodeHandle _nh;
		ros::Subscriber _jointstate_sub;
        ros::Publisher _ll_pub;

        //la rotazione(teta) dei 4 giunti di hip
		float _hblp;
		float _hbrp;
		float _hflp;
		float _hfrp;

		//knee joint

		float _kblp;
		float _kbrp;
		float _kflp;
		float _kfrp;



        //dimensione dei link
		float a1=0.315;
		float a2=0.315;

};

CIN_DIR::CIN_DIR() {

	_jointstate_sub = _nh.subscribe ("/dogbot/joint_states", 10, &CIN_DIR::calculate_leg_length_cb, this);
	
	_ll_pub = _nh.advertise<tesi::seleziona_gamba>("/data/len_leg",10);
}

void CIN_DIR::calculate_leg_length_cb(sensor_msgs::JointStateConstPtr js){
	
	//mi servono le 4 posizione dei giunti di hip e le 4 coppie applicate a questi
	/*
	_hblp = js->position[1]; //in questo modo prendo il secondo elemento del vettore position nello specifico:back_left_pitch_joint
	_hbrp = js->position[4];
	_hflp = js->position[7];
	_hfrp = js->position[10];

	_kblp = js->position[0]; //back_left_knee_joint
	_kbrp = js->position[3];
	_kflp = js->position[6];
	_kfrp = js->position[9];
    */

    float hp[4];
    float kp[4];
    float rl[4];
    float fx;
    float fz;
    string l[]={"bl","br","fl","fr"};
    
    for (int i = 0;i<4;i++){
        hp[i] = js->position[1 +3*i];
        kp[i] = js->position[3*i];
        fx = a1 * cos (hp[i]) + a2 * cos(hp[i] + kp[i]);
	    fz = a1 * sin (hp[i]) + a2 * sin(hp[i] + kp[i]);

        rl[i] = sqrt(pow(fx,2)+pow(fz,2));
	    cout<<"lunghezza leg "<<l[i]<<":"<<rl[i] <<endl;    


    }

    tesi::seleziona_gamba ll;
    ll.bl=rl[0];
    ll.br=rl[1];
    ll.fl=rl[2];
    ll.fr=rl[3];
    _ll_pub.publish(ll);

}


int main(int argc, char **argv){
	
	ros::init(argc, argv, "len_leg");
	
	CIN_DIR rs;
	
	
	ros::spin();
	
	return 0;

}
