#ifndef QUADRUPEDMODEL
#define QUADRUPEDMODEL

#include <cstdlib>
#include <iostream>
#include <cmath>
// Eigen headers 
//#include <Eigen/Core>
#include "eigen3/Eigen/Core"

// iDynTree headers
#include <iDynTree/Model/FreeFloatingState.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/ModelIO/ModelLoader.h>

// Helpers function to convert between 
// iDynTree datastructures
#include <iDynTree/Core/EigenHelpers.h>
#include "tesi/optimization.h"

class QUADRUPED
{
  public:
  
  //robot
   QUADRUPED();
   QUADRUPED(std::string modelFile);
  
  // update the robot state
	void update(Eigen::Matrix4d &eigenWorld_H_base, Eigen::VectorXd &eigenJointPos, Eigen::VectorXd &eigenJointVel, Eigen::Matrix<double,6,1> &eigenBasevel, Eigen::Vector3d &eigenGravity);			
  
  
  // get function
	Eigen::VectorXd getBiasMatrix();
	Eigen::VectorXd getGravityMatrix();
	Eigen::MatrixXd getMassMatrix();
  Eigen::MatrixXd getJacobian();
  Eigen::MatrixXd getBiasAcc();
  Eigen::MatrixXd getTransMatrix();

  Eigen::MatrixXd getTdotMatrix();
  
  Eigen::VectorXd getBiasMatrixCOM();
	Eigen::VectorXd getGravityMatrixCOM();
	Eigen::MatrixXd getMassMatrixCOM();
  Eigen::MatrixXd getMassMatrixCOM_com();
  Eigen::MatrixXd getMassMatrixCOM_joints();
  Eigen::MatrixXd getJacobianCOM();
  Eigen::MatrixXd getJacobianCOM_linear();
	Eigen::MatrixXd getBiasAccCOM();
  Eigen::MatrixXd getBiasAccCOM_linear();
  Eigen::MatrixXd getCOMpos();
  Eigen::MatrixXd getCOMvel();
  
  double getMass();
  int getDoFsnumber();

  private:
  
  // int for DoFs number
  unsigned int n;
  
  // Total mass of the robot
  double robot_mass;

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

  // Position vector
  iDynTree::VectorDynSize  q;

  // Velocity vector
  iDynTree::VectorDynSize  dq;

  // Joints limit vector

  iDynTree::VectorDynSize  qmin;
  iDynTree::VectorDynSize  qmax;
  
  // Center of Mass Position

  iDynTree::Vector6 CoM;

  // Center of mass velocity

  iDynTree::Vector6 CoM_vel;
  
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
  iDynTree::MatrixDynSize T_inv_dot;
  
  //Model
  iDynTree::Model model;
  iDynTree::ModelLoader mdlLoader;
  
   //Mass matrix in CoM representation
  iDynTree::FreeFloatingMassMatrix MassMatrixCOM;
  
  
  //Bias Matrix in CoM representation
  iDynTree::VectorDynSize BiasCOM;
  
  //Gravity Matrix in CoM representation
  iDynTree::MatrixDynSize GravMatrixCOM;
  
  // Jacobian in CoM representation
  iDynTree::MatrixDynSize JacCOM;

  //Jacobian in CoM representation (only linear part)
  iDynTree::MatrixDynSize JacCOM_lin;

  // Bias acceleration J_dot*q_dot in CoM representation
  iDynTree::MatrixDynSize JdqdCOM;

   // Bias acceleration J_dot*q_dot in CoM representation
  iDynTree::MatrixDynSize JdqdCOM_lin;
  
  //Create the robot
  void createrobot(std::string modelFile);
  
  // Compute the Jacobian
  void  computeJac();

  // Compute matrix transformation T needed to recompute matrices/vecotor after the coordinate transform to the CoM
  void computeTransformation(Eigen::VectorXd Vel_);
  
  // Compute bias acceleration J_dot*q_dot
  void computeJacDotQDot();
  
  // Compute Jacobian time derivative
  void computeJacDot(Eigen::VectorXd Vel_);
  
  //Compute partial derivative
  Eigen::VectorXd  getPartialDerivativeJac(const Eigen::MatrixXd Jacobian, const unsigned int& joint_idx,  const unsigned int& column_idx);
 };

#endif
   


