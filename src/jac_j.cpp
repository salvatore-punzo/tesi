
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
class JACJ {
    public:
        JACJ(Eigen::VectorXd q);
        void run();
        void Jj( Eigen::VectorXd in1 );
        Eigen::MatrixXd J0=Eigen::MatrixXd::Zero(12,12);
    private:
        ros::NodeHandle _nh;
        //double *q;
        
};   

JACJ::JACJ(Eigen::VectorXd q) {
		
		/*_as.registerPreemptCallback( boost::bind(&SURV::preemptCb, this) );
    _laser_sub = _nh.subscribe("/laser/scan",0,&SURV::laser_cb,this);*/
    JACJ::Jj(q);
  
    /*_cmd_vel_pub = _nh.advertise< geometry_msgs::Twist>("/cmd_vel", 0);
    _as.start();*/
}
   
  
void JACJ::Jj( Eigen::VectorXd  in1){ 
  

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
 double  t33 = t2*t19;
 double  t34 = t4*t17;
 double  t35 = t5*t18;
 double  t36 = t8*t18;
 double  t37 = t11*t18;
 double  t38 = t14*t18;
 double  t39 = t17*t19;
 double  t40 = t18*t20;
 double  t41 = t18*t23;
 double  t42 = t18*t26;
 double  t43 = t18*t29;
 double  t45 = t2*t3*t5;
 double  t46 = t3*t4*t5;
 double  t47 = t2*t3*t8;
 double  t48 = t3*t4*t8;
 double  t49 = t2*t3*t11;
 double  t50 = t3*t4*t11;
 double  t51 = t2*t3*t14;
 double  t52 = t3*t4*t14;
 double  t54 = t2*t3*t20;
 double  t55 = t3*t5*t17;
 double  t56 = t3*t4*t20;
 double  t57 = t2*t3*t23;
 double  t58 = t3*t6*t19;
 double  t59 = t3*t8*t17;
 double  t60 = t3*t4*t23;
 double  t61 = t2*t3*t26;
 double  t62 = t3*t9*t19;
 double  t63 = t3*t11*t17;
 double  t64 = t3*t4*t26;
 double  t65 = t2*t3*t29;
 double  t66 = t3*t12*t19;
 double  t67 = t3*t14*t17;
 double  t68 = t3*t4*t29;
 double  t69 = t3*t15*t19;
 double  t72 = t3*t17*t20;
 double  t73 = t3*t17*t23;
 double  t74 = t3*t19*t21;
 double  t75 = t3*t17*t26;
 double  t76 = t3*t19*t24;
 double  t77 = t3*t17*t29;
 double  t78 = t3*t19*t27;
 double  t79 = t3*t19*t30;
 double  t44 = t18*t39;
 double  t53 = t18*t32;
 double  t70 = t18*t33;
 double  t71 = t18*t34;
 double  t81 = -t45;
 double  t82 = -t47;
 double  t83 = -t50;
 double  t84 = -t52;
 double  t86 = -t56;
 double  t87 = -t60;
 double  t88 = -t61;
 double  t89 = -t62;
 double  t90 = -t63;
 double  t91 = -t65;
 double  t92 = -t67;
 double  t93 = -t69;
 double  t94 = -t72;
 double  t95 = -t73;
 double  t96 = -t74;
 double  t97 = -t78;
 double  t98 = t40+t46;
 double  t99 = t41+t48;
 double  t100 = t37+t64;
 double  t101 = t38+t68;
 double  t80 = -t44;
 double  t85 = -t53;
 double  t102 = t33+t71;
 double  t103 = t34+t70;
 double  t104 = t35+t86;
 double  t105 = t36+t87;
 double  t106 = t42+t83;
 double  t107 = t43+t84;
 double  t110 = t12*t100;
 double  t111 = t15*t101;
 double  t120 = t27*t100;
 double  t121 = t30*t101;
 double  t108 = t32+t80;
 double  t109 = t39+t85;
 double  t112 = t5*t102;
 double  t113 = t6*t103;
 double  t114 = t8*t102;
 double  t115 = t9*t103;
 double  t116 = t11*t102;
 double  t117 = t12*t103;
 double  t118 = t14*t102;
 double  t119 = t15*t103;
 double  t122 = t20*t102;
 double  t123 = t21*t103;
 double  t124 = t23*t102;
 double  t125 = t24*t103;
 double  t126 = t26*t102;
 double  t127 = t27*t103;
 double  t128 = t29*t102;
 double  t129 = t30*t103;
 double  t130 = t6*t104;
 double  t131 = t9*t105;
 double  t138 = t21*t104;
 double  t143 = t24*t105;
 double  t162 = t66+t120;
 double  t163 = t79+t111;
 double  t174 = t97+t110;
 double  t175 = t93+t121;
 double  t201 = t16*(t69-t121)*(-3.0/1.0E+1);
 double  t203 = t28*(t78-t110)*(-3.0/1.0E+1);
 double  t132 = t5*t109;
 double  t133 = t6*t108;
 double  t134 = t8*t109;
 double  t135 = t9*t108;
 double  t136 = -t115;
 double  t137 = t11*t109;
 double  t139 = t12*t108;
 double  t140 = -t117;
 double  t141 = t14*t109;
 double  t142 = t15*t108;
 double  t144 = t20*t109;
 double  t145 = t21*t108;
 double  t146 = -t123;
 double  t147 = t23*t109;
 double  t148 = t24*t108;
 double  t149 = t26*t109;
 double  t150 = t27*t108;
 double  t151 = t29*t109;
 double  t152 = t30*t108;
 double  t153 = -t129;
 double  t158 = t55+t122;
 double  t159 = t59+t124;
 double  t160 = t75+t116;
 double  t161 = t77+t118;
 double  t164 = t94+t112;
 double  t165 = t95+t114;
 double  t166 = t90+t126;
 double  t167 = t92+t128;
 double  t170 = t58+t138;
 double  t173 = t76+t131;
 double  t182 = t96+t130;
 double  t185 = t89+t143;
 double  t188 = -t12*(t63-t126);
 double  t190 = -t15*(t67-t128);
 double  t192 = -t27*(t63-t126);
 double  t193 = -t30*(t67-t128);
 double  t196 = t13*t162*(3.0/1.0E+1);
 double  t199 = t31*t163*(3.0/1.0E+1);
 double  t204 = t10*(t62-t143)*(-3.0/1.0E+1);
 double  t205 = t22*(t74-t130)*(-3.0/1.0E+1);
 double  t154 = -t135;
 double  t155 = -t139;
 double  t156 = -t145;
 double  t157 = -t152;
 double  t168 = t54+t132;
 double  t169 = t57+t134;
 double  t171 = t49+t149;
 double  t172 = t51+t151;
 double  t176 = t6*t158;
 double  t177 = t9*t159;
 double  t178 = t21*t158;
 double  t179 = t24*t159;
 double  t180 = t81+t144;
 double  t181 = t82+t147;
 double  t183 = t88+t137;
 double  t184 = t91+t141;
 double  t194 = -t6*(t45-t144);
 double  t195 = -t9*(t47-t147);
 double  t197 = -t21*(t45-t144);
 double  t198 = -t24*(t47-t147);
 double  t200 = t7*t170*(3.0/1.0E+1);
 double  t202 = t25*t173*(3.0/1.0E+1);
 double  t216 = t150+t188;
 double  t217 = t142+t193;
 double  t234 = t10*(t115+t24*(t47-t147))*(-3.0/1.0E+1);
 double  t235 = t22*(t123+t6*(t45-t144))*(-3.0/1.0E+1);
 double  t236 = t13*(t139+t27*(t63-t126))*(-3.0/1.0E+1);
 double  t237 = t31*(t152+t15*(t67-t128))*(-3.0/1.0E+1);
 double  t186 = t12*t171;
 double  t187 = t15*t172;
 double  t189 = t27*t171;
 double  t191 = t30*t172;
 double  t208 = t133+t178;
 double  t209 = t148+t177;
 double  t210 = t113+t197;
 double  t211 = t125+t195;
 double  t214 = t156+t176;
 double  t215 = t154+t179;
 double  t218 = t146+t194;
 double  t219 = t136+t198;
 double  t220 = t155+t192;
 double  t221 = t157+t190;
 double  t228 = t10*(t135-t179)*(-3.0/1.0E+1);
 double  t230 = t16*t217*(3.0/1.0E+1);
 double  t232 = t22*(t145-t176)*(-3.0/1.0E+1);
 double  t233 = t28*t216*(3.0/1.0E+1);
 double  t206 = t127+t186;
 double  t207 = t119+t191;
 double  t212 = t140+t189;
 double  t213 = t153+t187;
 double  t223 = t7*t208*(3.0/1.0E+1);
 double  t225 = t25*t209*(3.0/1.0E+1);
 double  t226 = t7*t210*(3.0/1.0E+1);
 double  t227 = t13*(t117-t189)*(-3.0/1.0E+1);
 double  t229 = t25*t211*(3.0/1.0E+1);
 double  t231 = t31*(t129-t187)*(-3.0/1.0E+1);
 double  t222 = t16*t207*(3.0/1.0E+1);
 double  t224 = t28*t206*(3.0/1.0E+1);
  J0(0,0) = t35*(-7.9E+1/8.0E+2)+t56*(7.9E+1/8.0E+2)+t6*t98*(6.3E+1/2.0E+2)+t6*t7*t98*(3.0/1.0E+1)+t21*t22*t98*(3.0/1.0E+1);
  J0(0,1) = t58*(6.3E+1/2.0E+2)+t138*(6.3E+1/2.0E+2)+t200+t22*(t74-t130)*(3.0/1.0E+1);
  J0(0,2) = -t200+t205;
  J0(0,3)=0;
  J0(0,4)=0;
  J0(0,5)=0;
  J0(0,6)=0;
  J0(0,7)=0;
  J0(0,8)=0;
  J0(0,9)=0;
  J0(0,10)=0;
  J0(0,11)=0;
  J0(1,0) = t55*(7.9E+1/8.0E+2)+t122*(7.9E+1/8.0E+2)-t6*(t72-t112)*(6.3E+1/2.0E+2)-t6*t7*(t72-t112)*(3.0/1.0E+1)-t21*t22*(t72-t112)*(3.0/1.0E+1);
  J0(1,1)  = t133*(-6.3E+1/2.0E+2)-t178*(6.3E+1/2.0E+2)-t223+t232;
  J0(1,2)  = t223+t22*(t145-t176)*(3.0/1.0E+1);
  J0(1,3)=0;
  J0(1,4)=0;
  J0(1,5)=0;
  J0(1,6)=0;
  J0(1,7)=0;
  J0(1,8)=0;
  J0(1,9)=0;
  J0(1,10)=0;
  J0(1,11)=0;
	
  J0(2,0)  = t45*(-7.9E+1/8.0E+2)+t144*(7.9E+1/8.0E+2)+t6*t168*(6.3E+1/2.0E+2)+t6*t7*t168*(3.0/1.0E+1)+t21*t22*t168*(3.0/1.0E+1);
  J0(2,1) = t113*(-6.3E+1/2.0E+2)-t226+t235+t21*(t45-t144)*(6.3E+1/2.0E+2);
  J0(2,2) = t226+t22*(t123+t6*(t45-t144))*(3.0/1.0E+1);
  J0(2,3)=0;
  J0(2,4)=0;
  J0(2,5)=0;
  J0(2,6)=0;
  J0(2,7)=0;
  J0(2,8)=0;
  J0(2,9)=0;
  J0(2,10)=0;
  J0(2,11)=0;
	
  J0(3,0)=0;
  J0(3,1)=0;
  J0(3,2)=0;	
  J0(3,3) = t36*(7.9E+1/8.0E+2)-t60*(7.9E+1/8.0E+2)+t9*t99*(6.3E+1/2.0E+2)+t9*t10*t99*(3.0/1.0E+1)+t24*t25*t99*(3.0/1.0E+1);
  J0(3,4) = t62*(-6.3E+1/2.0E+2)+t143*(6.3E+1/2.0E+2)-t202+t204;
  J0(3,5) = t202+t10*(t62-t143)*(3.0/1.0E+1);
  J0(3,6)=0;
  J0(3,7)=0;
  J0(3,8)=0;
  J0(3,9)=0;
  J0(3,10)=0;
  J0(3,11)=0;
	
  J0(4,0)=0;
  J0(4,1)=0;
  J0(4,2)=0;
  J0(4,3) = t59*(-7.9E+1/8.0E+2)-t124*(7.9E+1/8.0E+2)-t9*(t73-t114)*(6.3E+1/2.0E+2)-t9*t10*(t73-t114)*(3.0/1.0E+1)-t24*t25*(t73-t114)*(3.0/1.0E+1);
  J0(4,4) = t135*(6.3E+1/2.0E+2)-t179*(6.3E+1/2.0E+2)+t225+t10*(t135-t179)*(3.0/1.0E+1);
  J0(4,5) = -t225+t228;
  J0(4,6)=0;
  J0(4,7)=0;
  J0(4,8)=0;
  J0(4,9)=0;
  J0(4,10)=0;
  J0(4,11)=0;
	
  J0(5,0)=0;
  J0(5,1)=0;
  J0(5,2)=0;
  J0(5,3) = t47*(7.9E+1/8.0E+2)-t147*(7.9E+1/8.0E+2)+t9*t169*(6.3E+1/2.0E+2)+t9*t10*t169*(3.0/1.0E+1)+t24*t25*t169*(3.0/1.0E+1);
  J0(5,4) = t115*(6.3E+1/2.0E+2)+t229+t10*(t115+t24*(t47-t147))*(3.0/1.0E+1)+t24*(t47-t147)*(6.3E+1/2.0E+2);
  J0(5,5) = -t229+t234;
  J0(5,6)=0;
  J0(5,7)=0;
  J0(5,8)=0;
  J0(5,9)=0;
  J0(5,10)=0;
  J0(5,11)=0;
	
  J0(6,0)=0;
  J0(6,1)=0;
  J0(6,2)=0;
  J0(6,3)=0;
  J0(6,4)=0;
  J0(6,5)=0;	
  J0(6,6) = t37*(7.9E+1/8.0E+2)+t64*(7.9E+1/8.0E+2)+t12*t106*(6.3E+1/2.0E+2)+t12*t13*t106*(3.0/1.0E+1)+t27*t28*t106*(3.0/1.0E+1);
  J0(6,7) = t66*(6.3E+1/2.0E+2)+t120*(6.3E+1/2.0E+2)+t196+t28*(t78-t110)*(3.0/1.0E+1);
  J0(6,8) = -t196+t203;
  J0(6,9)=0;
  J0(6,10)=0;
  J0(6,11)=0;
	
  J0(7,0)=0;
  J0(7,1)=0;
  J0(7,2)=0;
  J0(7,3)=0;
  J0(7,4)=0;
  J0(7,5)=0;
  J0(7,6) = t63*(-7.9E+1/8.0E+2)+t126*(7.9E+1/8.0E+2)-t12*t160*(6.3E+1/2.0E+2)-t12*t13*t160*(3.0/1.0E+1)-t27*t28*t160*(3.0/1.0E+1);
  J0(7,7) = t139*(-6.3E+1/2.0E+2)-t233+t236-t27*(t63-t126)*(6.3E+1/2.0E+2);
  J0(7,8) = t233+t13*(t139+t27*(t63-t126))*(3.0/1.0E+1);
  J0(7,9)=0;
  J0(7,10)=0;
  J0(7,11)=0;
	
  J0(8,0)=0;
  J0(8,1)=0;
  J0(8,2)=0;
  J0(8,3)=0;
  J0(8,4)=0;
  J0(8,5)=0;	
  J0(8,6) = t49*(7.9E+1/8.0E+2)+t149*(7.9E+1/8.0E+2)+t12*(t61-t137)*(6.3E+1/2.0E+2)+t12*t13*(t61-t137)*(3.0/1.0E+1)+t27*t28*(t61-t137)*(3.0/1.0E+1);
  J0(8,7) = t117*(-6.3E+1/2.0E+2)+t189*(6.3E+1/2.0E+2)-t224+t227;
  J0(8,8) = t224+t13*(t117-t189)*(3.0/1.0E+1);
  J0(8,9)=0;
  J0(8,10)=0;
  J0(8,11)=0;
	
  J0(9,0)=0;
  J0(9,1)=0;
  J0(9,2)=0;
  J0(9,3)=0;
  J0(9,4)=0;
  J0(9,5)=0;	
  J0(9,6)=0;
  J0(9,7)=0;
  J0(9,8)=0;	
  J0(9,9) = t38*(-7.9E+1/8.0E+2)-t68*(7.9E+1/8.0E+2)+t15*t107*(6.3E+1/2.0E+2)+t15*t16*t107*(3.0/1.0E+1)+t30*t31*t107*(3.0/1.0E+1);
  J0(9,10) = t69*(-6.3E+1/2.0E+2)+t121*(6.3E+1/2.0E+2)-t199+t201;
  J0(9,11) = t199+t16*(t69-t121)*(3.0/1.0E+1);
	
  J0(10,0)=0;
  J0(10,1)=0;
  J0(10,2)=0;
  J0(10,3)=0;
  J0(10,4)=0;
  J0(10,5)=0;	
  J0(10,6)=0;
  J0(10,7)=0;
  J0(10,8)=0;
  J0(10,9) = t67*(7.9E+1/8.0E+2)-t128*(7.9E+1/8.0E+2)-t15*t161*(6.3E+1/2.0E+2)-t15*t16*t161*(3.0/1.0E+1)-t30*t31*t161*(3.0/1.0E+1);
  J0(10,10) = t142*(6.3E+1/2.0E+2)+t230+t31*(t152+t15*(t67-t128))*(3.0/1.0E+1)-t30*(t67-t128)*(6.3E+1/2.0E+2);
  J0(10,11) = -t230+t237;
	
  J0(11,0)=0;
  J0(11,1)=0;
  J0(11,2)=0;
  J0(11,3)=0;
  J0(11,4)=0;
  J0(11,5)=0;	
  J0(11,6)=0;
  J0(11,7)=0;
  J0(11,8)=0;
  J0(11,9) = t51*(-7.9E+1/8.0E+2)-t151*(7.9E+1/8.0E+2)+t15*(t65-t141)*(6.3E+1/2.0E+2)+t15*t16*(t65-t141)*(3.0/1.0E+1)+t30*t31*(t65-t141)*(3.0/1.0E+1);
  J0(11,10) = t119*(6.3E+1/2.0E+2)+t191*(6.3E+1/2.0E+2)+t222+t31*(t129-t187)*(3.0/1.0E+1);
  J0(11,11) = -t222+t231;
  }
