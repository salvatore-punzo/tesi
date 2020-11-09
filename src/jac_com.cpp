#include <cmath>

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"
#include "boost/thread.hpp"
#include <actionlib/server/simple_action_server.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Accel.h"
#include <tf/tf.h>
#include <eigen3/Eigen/Dense>



using namespace std;
class JACCOM {
    public:
        JACCOM(Eigen::VectorXd q);
        void run();
        void Jcomq( Eigen::VectorXd in1 );
        Eigen::MatrixXd A0;
    private:
        ros::NodeHandle _nh;
        //double *q;
        
};   

JACCOM::JACCOM(Eigen::VectorXd q) {
		
		/*_as.registerPreemptCallback( boost::bind(&SURV::preemptCb, this) );
    _laser_sub = _nh.subscribe("/laser/scan",0,&SURV::laser_cb,this);*/
    JACCOM::Jcomq(q);
  
    /*_cmd_vel_pub = _nh.advertise< geometry_msgs::Twist>("/cmd_vel", 0);
    _as.start();*/
}
  
  void JACCOM::Jcomq( Eigen::VectorXd in1 ){ 
  
 A0.resize(12,6);
 double  q4=in1[3];
 double  q5=in1[4];
 double  q6=in1[5];
 double  q7=in1[6]; 
 double  q8=in1[7];
 double  q9=in1[8];
 double  q10=in1[9];
 double  q11=in1[10];
 double  q12=in1[11];
 double  q13=in1[12];
 double  q14=in1[13];
 double  q15=in1[14];
 double  q16=in1[15];
 double  q17=in1[16];
 double  q18=in1[17];

 double  t2 = cos(q4);
 double  t3 = cos(q5);
 double  t4 = cos(q6);
 double  t5 = cos(q7);
 double  t6 = cos(q8);
 double  t7 = cos(q9);
 double  t8 = cos(q10);
 double  t9 = cos(q11);
 double  t10 = cos(q12);
 double  t11 = cos(q13);
 double  t12 = cos(q14);
 double  t13 = cos(q15);
 double  t14 = cos(q16);
 double  t15 = cos(q17);
 double  t16 = cos(q18);
 double  t17 = sin(q4);
 double  t18 = sin(q5);
 double  t19 = sin(q6);
 double  t20 = sin(q7);
 double  t21 = sin(q8);
 double  t22 = sin(q9);
 double  t23 = sin(q10);
 double  t24 = sin(q11);
 double  t25 = sin(q12);
 double  t26 = sin(q13);
 double  t27 = sin(q14);
 double  t28 = sin(q15);
 double  t29 = sin(q16);
 double  t30 = sin(q17);
 double  t31 = sin(q18);
 double  t32 = t2*t4;
 double  t33 = t3*t5;
 double  t34 = t3*t8;
 double  t35 = t3*t11;
 double  t36 = t3*t14;
 double  t37 = t2*t19;
 double  t38 = t4*t17;
 double  t39 = t17*t19;
 double  t46 = t2*t5*t18;
 double  t48 = t2*t8*t18;
 double  t50 = t2*t11*t18;
 double  t52 = t2*t14*t18;
 double  t56 = t5*t17*t18;
 double  t57 = t4*t18*t20;
 double  t58 = t8*t17*t18;
 double  t59 = t4*t18*t23;
 double  t60 = t11*t17*t18;
 double  t61 = t4*t18*t26;
 double  t62 = t14*t17*t18;
 double  t63 = t4*t18*t29;
 double  t81 = t3*t4*(2.3E+1/8.0E+1);
 double  t87 = t3*t19*(1.1E+1/1.25E+2);
 double  t88 = t4*t18*(1.1E+1/1.25E+2);
 double  t90 = t18*t19*(2.3E+1/8.0E+1);
 double  t40 = t18*t39;
 double  t41 = t2*t33;
 double  t42 = t2*t34;
 double  t43 = t2*t35;
 double  t44 = t2*t36;
 double  t45 = t18*t32;
 double  t47 = t17*t33;
 double  t49 = t17*t34;
 double  t51 = t17*t35;
 double  t53 = t17*t36;
 double  t54 = t18*t37;
 double  t55 = t18*t38;
 double  t65 = t3*t20*t32;
 double  t66 = t3*t23*t32;
 double  t67 = t3*t26*t32;
 double  t68 = t3*t29*t32;
 double  t69 = t3*t20*t38;
 double  t70 = t3*t23*t38;
 double  t71 = t3*t26*t38;
 double  t72 = t3*t29*t38;
 double  t78 = -t61;
 double  t79 = -t63;
 double  t80 = t32*(2.3E+1/8.0E+1);
 double  t82 = t32*(1.1E+1/1.25E+2);
 double  t83 = t37*(2.3E+1/8.0E+1);
 double  t84 = t38*(2.3E+1/8.0E+1);
 double  t85 = t37*(1.1E+1/1.25E+2);
 double  t86 = t38*(1.1E+1/1.25E+2);
 double  t89 = t39*(2.3E+1/8.0E+1);
 double  t91 = t39*(1.1E+1/1.25E+2);
 double  t97 = -t81;
 double  t103 = -t87;
 double  t104 = -t88;
 double  t106 = -t90;
 double  t110 = t33+t57;
 double  t111 = t34+t59;
 double  t126 = t3*t32*(-1.1E+1/1.25E+2);
 double  t127 = t3*t37*(-2.3E+1/8.0E+1);
 double  t130 = t3*t38*(-1.1E+1/1.25E+2);
 double  t132 = t3*t39*(-2.3E+1/8.0E+1);
 double  t64 = -t40;
 double  t73 = -t41;
 double  t74 = -t42;
 double  t75 = -t45;
 double  t76 = -t51;
 double  t77 = -t53;
 double  t92 = -t65;
 double  t93 = -t66;
 double  t94 = -t69;
 double  t95 = -t70;
 double  t96 = -t80;
 double  t98 = -t82;
 double  t99 = -t83;
 double  t100 = -t84;
 double  t101 = -t85;
 double  t102 = -t86;
 double  t105 = -t89;
 double  t107 = -t91;
 double  t108 = t40*(2.3E+1/8.0E+1);
 double  t109 = t40*(1.1E+1/1.25E+2);
 double  t112 = t37+t55;
 double  t113 = t38+t54;
 double  t114 = t3*t82;
 double  t115 = t3*t83;
 double  t116 = t45*(2.3E+1/8.0E+1);
 double  t117 = t45*(1.1E+1/1.25E+2);
 double  t118 = t3*t86;
 double  t119 = t54*(2.3E+1/8.0E+1);
 double  t120 = t3*t89;
 double  t121 = t55*(2.3E+1/8.0E+1);
 double  t122 = t54*(1.1E+1/1.25E+2);
 double  t123 = t55*(1.1E+1/1.25E+2);
 double  t136 = t35+t78;
 double  t137 = t36+t79;
 double  t144 = t50+t67;
 double  t145 = t52+t68;
 double  t146 = t60+t71;
 double  t147 = t62+t72;
 double  t124 = -t108;
 double  t125 = -t109;
 double  t128 = -t116;
 double  t129 = -t117;
 double  t131 = -t119;
 double  t133 = -t121;
 double  t134 = -t122;
 double  t135 = -t123;
 double  t138 = t32+t64;
 double  t139 = t39+t75;
 double  t140 = t20*t112;
 double  t141 = t23*t112;
 double  t142 = t26*t112;
 double  t143 = t29*t112;
 double  t152 = t46+t92;
 double  t153 = t48+t93;
 double  t154 = t56+t94;
 double  t155 = t58+t95;
 double  t148 = t20*t139;
 double  t149 = t23*t139;
 double  t150 = t26*t139;
 double  t151 = t29*t139;
 double  t156 = t47+t140;
 double  t157 = t49+t141;
 double  t158 = t76+t142;
 double  t159 = t77+t143;
 double  t160 = t43+t150;
 double  t161 = t44+t151;
 double  t162 = t73+t148;
 double  t163 = t74+t149;
  A0(0,0) = 1.0;
  A0(0,4) = t88+t106-t7*(t6*t110+t18*t19*t21)*(3.0/1.0E+1)-t22*(t21*t110-t6*t18*t19)*(3.0/1.0E+1)-t3*t20*(7.9E+1/8.0E+2)-t6*t110*(6.3E+1/2.0E+2)+t4*t5*t18*(7.9E+1/8.0E+2)-t18*t19*t21*(6.3E+1/2.0E+2);
  A0(0,5) = t81+t87+t7*(t3*t4*t21-t3*t6*t19*t20)*(3.0/1.0E+1)-t22*(t3*t4*t6+t3*t19*t20*t21)*(3.0/1.0E+1)+t19*t33*(7.9E+1/8.0E+2)+t3*t4*t21*(6.3E+1/2.0E+2)-t3*t6*t19*t20*(6.3E+1/2.0E+2);
  A0(1,1) = 1.0;
  A0(1,3) = t84+t91+t119+t129+t21*t113*(6.3E+1/2.0E+2)+t5*t139*(7.9E+1/8.0E+2)+t6*(t41-t148)*(6.3E+1/2.0E+2)+t7*(t21*t113+t6*(t41-t148))*(3.0/1.0E+1)-t22*(t6*t113-t21*(t41-t148))*(3.0/1.0E+1)+t2*t3*t20*(7.9E+1/8.0E+2);
  A0(1,4)= t120+t130-t7*(t6*t154-t3*t21*t39)*(3.0/1.0E+1)-t22*(t21*t154+t3*t6*t39)*(3.0/1.0E+1)-t33*t38*(7.9E+1/8.0E+2)-t6*t154*(6.3E+1/2.0E+2)-t17*t18*t20*(7.9E+1/8.0E+2)+t3*t21*t39*(6.3E+1/2.0E+2);
  A0(1,5) = t83+t98+t109+t121+t7*(t21*t112+t6*t20*t138)*(3.0/1.0E+1)-t22*(t6*t112-t20*t21*t138)*(3.0/1.0E+1)+t21*t112*(6.3E+1/2.0E+2)-t5*t138*(7.9E+1/8.0E+2)+t6*t20*t138*(6.3E+1/2.0E+2);
  A0(2,2) = 1.0;
  A0(2,3) = t96+t101+t108+t135-t5*t112*(7.9E+1/8.0E+2)-t21*t138*(6.3E+1/2.0E+2)+t6*t156*(6.3E+1/2.0E+2)-t7*(t21*t138-t6*t156)*(3.0/1.0E+1)+t22*(t6*t138+t21*t156)*(3.0/1.0E+1)+t3*t17*t20*(7.9E+1/8.0E+2);
  A0(2,4) = t114+t127+t7*(t6*t152-t3*t21*t37)*(3.0/1.0E+1)+t22*(t21*t152+t3*t6*t37)*(3.0/1.0E+1)+t32*t33*(7.9E+1/8.0E+2)+t6*t152*(6.3E+1/2.0E+2)+t2*t18*t20*(7.9E+1/8.0E+2)-t3*t21*t37*(6.3E+1/2.0E+2);
  A0(2,5) = t89+t102+t128+t134+t7*(t21*t139+t6*t20*t113)*(3.0/1.0E+1)-t22*(t6*t139-t20*t21*t113)*(3.0/1.0E+1)-t5*t113*(7.9E+1/8.0E+2)+t21*t139*(6.3E+1/2.0E+2)+t6*t20*t113*(6.3E+1/2.0E+2);
  A0(3,0) = 1.0;
  A0(3,4) = t104+t106-t10*(t9*t111-t18*t19*t24)*(3.0/1.0E+1)-t25*(t24*t111+t9*t18*t19)*(3.0/1.0E+1)+t3*t23*(7.9E+1/8.0E+2)-t9*t111*(6.3E+1/2.0E+2)-t4*t8*t18*(7.9E+1/8.0E+2)+t18*t19*t24*(6.3E+1/2.0E+2);
  A0(3,5) = t81+t103-t10*(t3*t4*t24+t3*t9*t19*t23)*(3.0/1.0E+1)+t25*(t3*t4*t9-t3*t19*t23*t24)*(3.0/1.0E+1)-t19*t34*(7.9E+1/8.0E+2)-t3*t4*t24*(6.3E+1/2.0E+2)-t3*t9*t19*t23*(6.3E+1/2.0E+2);
  A0(4,1) = 1.0;
  A0(4,3) = t84+t107+t117+t119-t24*t113*(6.3E+1/2.0E+2)-t8*t139*(7.9E+1/8.0E+2)+t9*(t42-t149)*(6.3E+1/2.0E+2)-t10*(t24*t113-t9*(t42-t149))*(3.0/1.0E+1)+t25*(t9*t113+t24*(t42-t149))*(3.0/1.0E+1)-t2*t3*t23*(7.9E+1/8.0E+2);
  A0(4,4) = t118+t120-t10*(t9*t155+t3*t24*t39)*(3.0/1.0E+1)-t25*(t24*t155-t3*t9*t39)*(3.0/1.0E+1)+t34*t38*(7.9E+1/8.0E+2)-t9*t155*(6.3E+1/2.0E+2)+t17*t18*t23*(7.9E+1/8.0E+2)-t3*t24*t39*(6.3E+1/2.0E+2);
  A0(4,5) = t82+t83+t121+t125-t10*(t24*t112-t9*t23*t138)*(3.0/1.0E+1)+t25*(t9*t112+t23*t24*t138)*(3.0/1.0E+1)-t24*t112*(6.3E+1/2.0E+2)+t8*t138*(7.9E+1/8.0E+2)+t9*t23*t138*(6.3E+1/2.0E+2);
  A0(5,2) = 1.0;
  A0(5,3) = t85+t96+t108+t123+t8*t112*(7.9E+1/8.0E+2)+t24*t138*(6.3E+1/2.0E+2)+t9*t157*(6.3E+1/2.0E+2)+t10*(t24*t138+t9*t157)*(3.0/1.0E+1)-t25*(t9*t138-t24*t157)*(3.0/1.0E+1)-t3*t17*t23*(7.9E+1/8.0E+2);
  A0(5,4) = t126+t127+t10*(t9*t153+t3*t24*t37)*(3.0/1.0E+1)+t25*(t24*t153-t3*t9*t37)*(3.0/1.0E+1)-t32*t34*(7.9E+1/8.0E+2)+t9*t153*(6.3E+1/2.0E+2)-t2*t18*t23*(7.9E+1/8.0E+2)+t3*t24*t37*(6.3E+1/2.0E+2);
  A0(5,5) = t86+t89+t122+t128-t10*(t24*t139-t9*t23*t113)*(3.0/1.0E+1)+t25*(t9*t139+t23*t24*t113)*(3.0/1.0E+1)+t8*t113*(7.9E+1/8.0E+2)-t24*t139*(6.3E+1/2.0E+2)+t9*t23*t113*(6.3E+1/2.0E+2);
  A0(6,0) = 1.0;
  A0(6,4) = t88+t90-t13*(t12*t136+t18*t19*t27)*(3.0/1.0E+1)-t28*(t27*t136-t12*t18*t19)*(3.0/1.0E+1)+t3*t26*(7.9E+1/8.0E+2)-t12*t136*(6.3E+1/2.0E+2)+t4*t11*t18*(7.9E+1/8.0E+2)-t18*t19*t27*(6.3E+1/2.0E+2);
  A0(6,5) = t87+t97+t13*(t3*t4*t27+t3*t12*t19*t26)*(3.0/1.0E+1)-t28*(t3*t4*t12-t3*t19*t26*t27)*(3.0/1.0E+1)+t19*t35*(7.9E+1/8.0E+2)+t3*t4*t27*(6.3E+1/2.0E+2)+t3*t12*t19*t26*(6.3E+1/2.0E+2);
  A0(7,1) = 1.0;
  A0(7,3) = t91+t100+t129+t131+t27*t113*(6.3E+1/2.0E+2)+t11*t139*(7.9E+1/8.0E+2)+t12*t160*(6.3E+1/2.0E+2)+t13*(t27*t113+t12*t160)*(3.0/1.0E+1)-t28*(t12*t113-t27*t160)*(3.0/1.0E+1)-t2*t3*t26*(7.9E+1/8.0E+2);
  A0(7,4) = t130+t132-t13*(t12*t146-t3*t27*t39)*(3.0/1.0E+1)-t28*(t27*t146+t3*t12*t39)*(3.0/1.0E+1)-t35*t38*(7.9E+1/8.0E+2)-t12*t146*(6.3E+1/2.0E+2)+t17*t18*t26*(7.9E+1/8.0E+2)+t3*t27*t39*(6.3E+1/2.0E+2);
  A0(7,5) = t98+t99+t109+t133+t13*(t27*t112-t12*t26*t138)*(3.0/1.0E+1)-t28*(t12*t112+t26*t27*t138)*(3.0/1.0E+1)+t27*t112*(6.3E+1/2.0E+2)-t11*t138*(7.9E+1/8.0E+2)-t12*t26*t138*(6.3E+1/2.0E+2);
  A0(8,2) = 1.0;
  A0(8,3) = t80+t101+t124+t135-t11*t112*(7.9E+1/8.0E+2)-t27*t138*(6.3E+1/2.0E+2)+t12*(t51-t142)*(6.3E+1/2.0E+2)-t13*(t27*t138-t12*(t51-t142))*(3.0/1.0E+1)+t28*(t12*t138+t27*(t51-t142))*(3.0/1.0E+1)-t3*t17*t26*(7.9E+1/8.0E+2);
  A0(8,4) = t114+t115+t13*(t12*t144-t3*t27*t37)*(3.0/1.0E+1)+t28*(t27*t144+t3*t12*t37)*(3.0/1.0E+1)+t32*t35*(7.9E+1/8.0E+2)+t12*t144*(6.3E+1/2.0E+2)-t2*t18*t26*(7.9E+1/8.0E+2)-t3*t27*t37*(6.3E+1/2.0E+2);
  A0(8,5) = t102+t105+t116+t134+t13*(t27*t139-t12*t26*t113)*(3.0/1.0E+1)-t28*(t12*t139+t26*t27*t113)*(3.0/1.0E+1)-t11*t113*(7.9E+1/8.0E+2)+t27*t139*(6.3E+1/2.0E+2)-t12*t26*t113*(6.3E+1/2.0E+2);
  A0(9,0) = 1.0;
  A0(9,4) = t90+t104-t16*(t15*t137-t18*t19*t30)*(3.0/1.0E+1)-t31*(t30*t137+t15*t18*t19)*(3.0/1.0E+1)-t3*t29*(7.9E+1/8.0E+2)-t15*t137*(6.3E+1/2.0E+2)-t4*t14*t18*(7.9E+1/8.0E+2)+t18*t19*t30*(6.3E+1/2.0E+2);
  A0(9,5) = t97+t103-t16*(t3*t4*t30-t3*t15*t19*t29)*(3.0/1.0E+1)+t31*(t3*t4*t15+t3*t19*t29*t30)*(3.0/1.0E+1)-t19*t36*(7.9E+1/8.0E+2)-t3*t4*t30*(6.3E+1/2.0E+2)+t3*t15*t19*t29*(6.3E+1/2.0E+2);
  A0(10,1) = 1.0;
  A0(10,3) = t100+t107+t117+t131-t30*t113*(6.3E+1/2.0E+2)-t14*t139*(7.9E+1/8.0E+2)+t15*t161*(6.3E+1/2.0E+2)-t16*(t30*t113-t15*t161)*(3.0/1.0E+1)+t31*(t15*t113+t30*t161)*(3.0/1.0E+1)+t2*t3*t29*(7.9E+1/8.0E+2);
  A0(10,4) = t118+t132-t16*(t15*t147+t3*t30*t39)*(3.0/1.0E+1)-t31*(t30*t147-t3*t15*t39)*(3.0/1.0E+1)+t36*t38*(7.9E+1/8.0E+2)-t15*t147*(6.3E+1/2.0E+2)-t17*t18*t29*(7.9E+1/8.0E+2)-t3*t30*t39*(6.3E+1/2.0E+2);
  A0(10,5) = t82+t99+t125+t133-t16*(t30*t112+t15*t29*t138)*(3.0/1.0E+1)+t31*(t15*t112-t29*t30*t138)*(3.0/1.0E+1)-t30*t112*(6.3E+1/2.0E+2)+t14*t138*(7.9E+1/8.0E+2)-t15*t29*t138*(6.3E+1/2.0E+2);
  A0(11,2) = 1.0;
  A0(11,3) = t80+t85+t123+t124+t14*t112*(7.9E+1/8.0E+2)+t30*t138*(6.3E+1/2.0E+2)+t15*(t53-t143)*(6.3E+1/2.0E+2)+t16*(t30*t138+t15*(t53-t143))*(3.0/1.0E+1)-t31*(t15*t138-t30*(t53-t143))*(3.0/1.0E+1)+t3*t17*t29*(7.9E+1/8.0E+2);
  A0(11,4) = t115+t126+t16*(t15*t145+t3*t30*t37)*(3.0/1.0E+1)+t31*(t30*t145-t3*t15*t37)*(3.0/1.0E+1)-t32*t36*(7.9E+1/8.0E+2)+t15*t145*(6.3E+1/2.0E+2)+t2*t18*t29*(7.9E+1/8.0E+2)+t3*t30*t37*(6.3E+1/2.0E+2);
  A0(11,5) = t86+t105+t116+t122-t16*(t30*t139+t15*t29*t113)*(3.0/1.0E+1)+t31*(t15*t139-t29*t30*t113)*(3.0/1.0E+1)+t14*t113*(7.9E+1/8.0E+2)-t30*t139*(6.3E+1/2.0E+2)-t15*t29*t113*(6.3E+1/2.0E+2);
  }

