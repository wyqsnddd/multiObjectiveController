# include <manipulatorTasks/motionTask.hpp>

void motionTask::updateReferences_(){
  // Update the desired position, velocity and acceleration.
  
  // Desired position 
  setTarget(referenceFramePtr_->getWorldTransform().translation());
  // Desired velocity
  velGoal_ = referenceFramePtr_->getLinearVelocity();
  // Desired acc
  accGoal_ = referenceFramePtr_->getLinearAcceleration();
  
}


void motionTask::update(){

  updateReferences_();

  dart::math::LinearJacobian Jv   = selectionMatrix_*eePtr_->getLinearJacobian();       // 3 x n
  dart::math::LinearJacobian dJv  = selectionMatrix_*eePtr_->getLinearJacobianDeriv();  // 3 x n

  Eigen::Vector3d x    = eePtr_->getTransform().translation();
  Eigen::VectorXd dq        = robotPtr_->getVelocities();                 // n x 1
	
  calcError(x);
  Eigen::Vector3d error = getError();

  std::cout<<"The position error is: "<< error.transpose()<<std::endl;

  Eigen::Vector3d tempConstant = (dJv + Kv_*Jv)*dq - Kv_*selectionMatrix_*velGoal_ + Kp_*error - selectionMatrix_*accGoal_;

  objQ_ = taskWeight_*Jv.transpose()*Jv;
  objP_ = taskWeight_*2*tempConstant.transpose()*Jv;
  objC_ = taskWeight_*tempConstant.transpose()*tempConstant;

  // 	std::cout<<"The position Q is: "<< objQ_ <<std::endl;
  // 	std::cout<<"The position P is: "<< objP_<<std::endl;
  //	std::cout<<"The position C is: "<< objC_<<std::endl;
}