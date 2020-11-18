#include "tesi/quadruped.h"

QUADRUPED::QUADRUPED()
{   
};

QUADRUPED::QUADRUPED(std::string modelFile)
{   //std::string modelFile="/home/salvatore/ros_ws/src/DogBotV4/ROS/src/dogbot_description/urdf/dogbot.urdf";
	createrobot(modelFile);
    model = kinDynComp.model();
    n=model.getNrOfDOFs();
    jointPos=iDynTree::VectorDynSize(n);
    baseVel=iDynTree::Twist();
    jointVel=iDynTree::VectorDynSize(n);
	coriol=iDynTree::VectorDynSize(n+6);
	grav=iDynTree::MatrixDynSize(n+6,1);
    idynMassMatrix=iDynTree::FreeFloatingMassMatrix(model) ;
    idynCOMJacobian=iDynTree::MatrixDynSize(3,n+6);
		
 
    std::cout<<"prova:"<<n<<std::endl;
    

}
 Eigen::VectorXd QUADRUPED::getCoriolisMatrix(){
        Eigen::VectorXd com = iDynTree::toEigen(coriol);
        return com;

    }

Eigen::MatrixXd QUADRUPED::getMassMatrix(){
        Eigen::MatrixXd Mass = iDynTree::toEigen(idynMassMatrix);
        return Mass;
}

void QUADRUPED::update (Eigen::Matrix4d world_H_base, Eigen::VectorXd jointPos_, Eigen::VectorXd jointVel_, Eigen::Matrix<double,6,1> basevel, Eigen::Vector3d gravity1)
{   
    iDynTree::Transform world_H_base_;
    iDynTree::VectorDynSize jointpos(n);
    iDynTree::Twist         baseVel_;
    iDynTree::VectorDynSize jointvel(n);
    iDynTree::Vector3       gravity_;
	 
	iDynTree::fromEigen(world_H_base_,world_H_base);
    iDynTree::toEigen(jointpos) = jointPos_;
    iDynTree::fromEigen(baseVel_,basevel);
    toEigen(jointvel) = jointVel_;
    toEigen(gravity_)  = gravity1;
	
	kinDynComp.setRobotState(world_H_base_,jointpos,
                             baseVel_,jointvel,gravity_);

	
	iDynTree::FreeFloatingGeneralizedTorques bias_force(model);
	iDynTree::FreeFloatingGeneralizedTorques grav_force(model);
	
	//Mass Matrix
	kinDynComp.getFreeFloatingMassMatrix(idynMassMatrix); 
	//Coriolis + gravitational terms
	kinDynComp.generalizedBiasForces(bias_force);
	Eigen::Matrix<double,6,1> bias_base= iDynTree::toEigen(bias_force.baseWrench());
	Eigen::Matrix<double,12,1> bias_joint= iDynTree::toEigen(bias_force.jointTorques());
	Eigen::Matrix<double,18,1> eigenbias;
	eigenbias << bias_base, 
	             bias_joint;
//	std::cout<<"gbias"<<eigenbias<<std::endl;
	iDynTree::toEigen(coriol)=eigenbias;
	//Gravitational term
	kinDynComp.generalizedGravityForces(grav_force);
	Eigen::Matrix<double,6,1> grav_base= iDynTree::toEigen(grav_force.baseWrench());
	Eigen::Matrix<double,12,1> grav_joint= iDynTree::toEigen(grav_force.jointTorques());
	Eigen::Matrix<double,18,1> eigengrav;
	eigengrav << grav_base, 
	             grav_joint;
//	std::cout<<"grav"<<eigengrav<<std::endl;
	iDynTree::toEigen(grav)=eigengrav;

   
	
}

void QUADRUPED::createrobot(std::string modelFile)
{  bool ok = mdlLoader.loadModelFromFile(modelFile);

    if( !ok )
    {
        std::cerr << "KinDynComputationsWithEigen: impossible to load model from " << modelFile << std::endl;
        return ;
    }
   
    // Create a KinDynComputations class from the model
    
    ok = kinDynComp.loadRobotModel(mdlLoader.model());

    if( !ok )
    {
        std::cerr << "KinDynComputationsWithEigen: impossible to load the following model in a KinDynComputations class:" << std::endl
                  << mdlLoader.model().toString() << std::endl;
        return ;
    }
    
  

}


