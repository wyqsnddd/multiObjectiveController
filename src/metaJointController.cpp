# include <jointControllers/metaJointController.hpp>


Eigen::VectorXd metaJointController::computedTorque_(const Eigen::VectorXd & acc){

	int nDof = robotPtr_->getNumDofs();
	Eigen::VectorXd tau(nDof);
	

	Eigen::VectorXd q = robotPtr_->getPositions();
	Eigen::VectorXd dq = robotPtr_->getVelocities(); 

	Eigen::VectorXd predict_dq = dq + dt_*acc; 
	Eigen::VectorXd predict_q = q + dt_*predict_dq; 


	Eigen::VectorXd q_error = q - predict_q;
	Eigen::VectorXd dq_error = dq - predict_dq;


	tau = robotPtr_->getMassMatrix()*(acc - Kv_*dq_error - Kp_*q_error ) + robotPtr_->getCoriolisAndGravityForces ();

	return tau;


}
