# include <controllers/gravityCompensationController.hpp>
Eigen::VectorXd gravityCompensationController::computeTorques(){
	int nDof = robotPtr_->getNumDofs();
	Eigen::VectorXd tau(nDof);
	//tau = Eigen::VectorXd::Zero(nDof);
	tau.setZero();

	if(!enabled_){
		return tau;	
	}else{

		Eigen::VectorXd Cg   = robotPtr_->getCoriolisAndGravityForces();
		tau = Cg;
		/*
		for (int ii = 0; ii< robotPtr_->getNumBodyNodes(); ii++){
			dart::dynamics::BodyNode* tempBodyPtr = robotPtr_->getBodyNode(ii);
			double m = tempBodyPtr->getMass();
			if(tempBodyPtr->getNumDependentDofs () > 0){
				dart::math::LinearJacobian tempJacobian = tempBodyPtr->getLinearJacobian(tempBodyPtr->getLocalCOM());
				tau += -tempJacobian.transpose()*(m*gravity_);
			}
		}
		*/
		//std::cout<<"The computed torque is: "<<tau<<std::endl;
		return tau;

	}

}
