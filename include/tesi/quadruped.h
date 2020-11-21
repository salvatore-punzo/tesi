//#ifndef QUADRUPED
//#define QUADRUPED

#include <cstdlib>
#include <iostream>
// Eigen headers 
#include <Eigen/Core>

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
  
  // update the robot state
	void update(Eigen::Matrix4d eigenWorld_H_base, Eigen::VectorXd eigenJointPos, Eigen::VectorXd eigenJointVel, Eigen::Matrix<double,6,1> eigenBasevel, Eigen::Vector3d eigenGravity);			
  
  // get function
	Eigen::VectorXd getBiasMatrix();
	Eigen::VectorXd getGravityMatrix();
	Eigen::MatrixXd getMassMatrix();
  Eigen::MatrixXd getJacobian();
  Eigen::MatrixXd getBiasAcc();
  Eigen::MatrixXd getTransMatrix();
  Eigen::VectorXd getBiasMatrixCOM();
	Eigen::VectorXd getGravityMatrixCOM();
	Eigen::MatrixXd getMassMatrixCOM();
  Eigen::MatrixXd getJacobianCOM();
	
  private:
  
  // int for DoFs number
  unsigned int n;
  
  // KinDynComputations element
  iDynTree::KinDynComputations kinDynComp;
  
  // world to floating base transformation
  iDynTree::Transform world_H_base;
  
  // Joint position
  iDynTree::VectorDynSize jointPos;
  
  // Floating base velocity
  iDynTree::Twist         baseVel;
  
  // Joint velocity
  iDynTree::VectorDynSize jointVel;
  
  // Gravity acceleration
  iDynTree::Vector3       gravity; 
  
  //Mass matrix
  iDynTree::FreeFloatingMassMatrix MassMatrix;
  
  //Bias Matrix
  iDynTree::VectorDynSize Bias;
  
  //Gravity Matrix
  iDynTree::MatrixDynSize GravMatrix;
  
  // Jacobian
  iDynTree::MatrixDynSize Jac;
  
  //CoM Jacobian
  iDynTree::MatrixDynSize Jcom;
  
  // Bias acceleration J_dot*q_dot
  iDynTree::MatrixDynSize Jdqd;
  
  // Transformation Matrix
  iDynTree::MatrixDynSize T;
  
  // Transformation matrix time derivative
  iDynTree::MatrixDynSize Tdot;
  
  //Model
  iDynTree::Model model;
  iDynTree::ModelLoader mdlLoader;
  
  //Create the robot
  void createrobot(std::string modelFile);
  
  // Compute the Jacobian
  void  computeJac();
  
   //Mass matrix in CoM representation
  iDynTree::FreeFloatingMassMatrix MassMatrixCOM;
  
  //Bias Matrix in CoM representation
  iDynTree::VectorDynSize BiasCOM;
  
  //Gravity Matrix in CoM representation
  iDynTree::MatrixDynSize GravMatrixCOM;
  
  // Jacobian in CoM representation
  iDynTree::MatrixDynSize JacCOM;
  
  
  // Compute matrix transformation T needed to recompute matrices/vecotor after the coordinate transform to the CoM
  void computeTransformation();
  
  // Compute Jacobian time derivative
  void computeJacDotQDot();
  
  //Compute partial derivative
 // Eigen::VectorXd  getPartialDerivativeJac(const Eigen::MatrixXd Jacobian, const unsigned int& joint_idx,  const unsigned int& column_idx);
 };

//#endif
   


