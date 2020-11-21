//#ifndef QUADRUPED
//#define QUADRUPED

#include <cstdlib>
#include <iostream>
// Eigen headers 
#include "eigen3/Eigen/Core"

// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>

// Helpers function to convert between 
// iDynTree datastructures
#include <iDynTree/Core/EigenHelpers.h>

class QUADRUPED
{
  public:
  
  //robot
    QUADRUPED();
  QUADRUPED(std::string modelFile);
    //unsigned int getNrJnts();
    //unsigned int getNrSgmts();
    //unsigned int test;
  // joints
  Eigen::VectorXd getCoriolisMatrix(); //modifica sp
  Eigen::MatrixXd getMassMatrix();
  /*Eigen::MatrixXd getJntLimits();
  Eigen::MatrixXd getJsim();
  Eigen::MatrixXd getCoriolisMatrix();
  Eigen::VectorXd getCoriolis();
  Eigen::VectorXd getGravity();
  Eigen::VectorXd getJntValues();
  Eigen::VectorXd getJntVelocities();
  KDL::ChainIdSolver_RNE* idSolver_;
  Eigen::VectorXd getID(const KDL::JntArray &q,
                        const KDL::JntArray &q_dot,
                        const KDL::JntArray &q_dotdot,
                        const KDL::Wrenches &f_ext);*/
	void update(Eigen::Matrix4d world_H_base, Eigen::VectorXd jointPos_, Eigen::VectorXd jointVel_, Eigen::Matrix<double,6,1> baseVel, Eigen::Vector3d gravity);					
				
	
	
  private:
  unsigned int n;
  
  
  iDynTree::KinDynComputations kinDynComp;
  iDynTree::Transform world_H_base;
  iDynTree::VectorDynSize jointPos;
  iDynTree::Twist         baseVel;
  iDynTree::VectorDynSize jointVel;
  iDynTree::Vector3       gravity; 
  iDynTree::FreeFloatingMassMatrix idynMassMatrix;
  iDynTree::VectorDynSize coriol;
  iDynTree::MatrixDynSize grav;
  
  iDynTree::MatrixDynSize idynCOMJacobian;
  
  iDynTree::Model model;
  iDynTree::ModelLoader mdlLoader;
  
  void createrobot(std::string modelFile);
  
 };

//#endif
   


