# include <controllers/metaController.hpp>

metaController::metaController(const dart::dynamics::SkeletonPtr & skelPtr, const pt::ptree configurationDataTree){

	enabled_ = true;
	robotPtr_ = skelPtr;
	assert(robotPtr_ != nullptr);


	configurationDataTree_ = configurationDataTree;

	controlMode_ = configurationDataTree_.get<int>("jointControlConfiguration.controlMode", 0);
	switch(readControlMode()){
		case 0:	
			// For computed torque control: 
			jointControllerPtr_ = std::make_shared<metaJointController>(robotPtr_, configurationDataTree_ );
			break;
		/* 
		 case 1:	
			// For joint velocity control
			return jointVelocityControl_(acc);
			break;
			*/
		default:
			std::cout<<"The controller mode is not set!"<<std::endl;
			throw std::runtime_error("The controller mode is not set!");
	}

}

/*
Eigen::VectorXd metaController::computeTorques(){
	int nDof = robotPtr_->getNumDofs();
	Eigen::VectorXd tau(nDof);
	tau = Eigen::VectorXd::Zero(nDof);
	tau.setZero();

	if(!enabled_){
		return tau;
	}else{

		Eigen::VectorXd Cg   = robotPtr_->getCoriolisAndGravityForces();
		tau = Cg;
		//std::cout<<"The computed torque is: "<<tau<<std::endl;
		return tau;

	}

}
*/
/*
Eigen::VectorXd metaController::jointVelocityControl_(const Eigen::VectorXd &acc){
	// The current implementation of joint velocity control is wrong, I need to re-exam this point. 

	double Kv = 2;
	int nDof = robotPtr_->getNumDofs();
	Eigen::VectorXd tau(nDof);
	double dt = robotPtr_->getTimeStep(); 
	

	Eigen::VectorXd accError = (robotPtr_->getAccelerations() - acc);

	tau = robotPtr_->getForces() - Kv*accError*dt; 
	
	return tau;

}

Eigen::VectorXd metaController::computedTorque_(const Eigen::VectorXd &acc){

	double Kp = 10;
	double Kv = 2;
	int nDof = robotPtr_->getNumDofs();
	Eigen::VectorXd tau(nDof);
	double dt = robotPtr_->getTimeStep(); 
	

	Eigen::VectorXd q = robotPtr_->getPositions();
	Eigen::VectorXd dq = robotPtr_->getVelocities(); 

	Eigen::VectorXd predict_dq = dq + dt*acc; 
	Eigen::VectorXd predict_q = q + dt*predict_dq; 


	Eigen::VectorXd q_error = q - predict_q;
	Eigen::VectorXd dq_error = dq - predict_dq;


	tau = robotPtr_->getMassMatrix()*(acc - Kv*dq_error - Kp*q_error ) + robotPtr_->getCoriolisAndGravityForces ();

	return tau;

}
*/
/*
Eigen::VectorXd metaController::accToTorque(const Eigen::VectorXd &acc){

	switch(readControlMode()){
		case 0:	
			return computedTorque_(acc);
		case 1:	
			return jointVelocityControl_(acc);
		default:
			std::cout<<"The controller mode is not set!"<<std::endl;
			throw std::runtime_error("The controller mode is not set!");
	}
}
*/
