# include <manipulatorTasks/linearVelocityTask.hpp>


void linearVelocityTask::update(){

	dart::math::LinearJacobian Jv   = selectionMatrix_*eePtr_->getLinearJacobian();       // 3 x n
	dart::math::LinearJacobian dJv  = selectionMatrix_*eePtr_->getLinearJacobianDeriv();  // 3 x n

	//Eigen::Vector3d x    = eePtr_->getTransform().translation();
	Eigen::Vector3d dx   = eePtr_->getLinearVelocity();
	Eigen::VectorXd dq        = robotPtr_->getVelocities();                 // n x 1
	
	calcError(dx);
	Eigen::Vector3d error = getError();


	std::cout<<"The linear velocity error is: "<< error.transpose()<<std::endl;

	Eigen::Vector3d tempConstant = (dJv + Kp_*Jv)*dq - Kp_*getTarget();

	objQ_ = taskWeight_*Jv.transpose()*Jv;
	objP_ = taskWeight_*2*tempConstant.transpose()*Jv;
	objC_ = taskWeight_*tempConstant.transpose()*tempConstant;

}
