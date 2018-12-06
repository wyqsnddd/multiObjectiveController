# include <manipulatorTasks/orientationTask.hpp>

void orientationTask::update(){

	Eigen::Matrix3d currentRotation =  eePtr_->getTransform().rotation();
	Eigen::Quaterniond currentQuat = Eigen::Quaterniond(currentRotation);
	calcQuatError(currentQuat);

	Eigen::MatrixXd block_one = desiredQuatMatrix_*conjugate_operation_Matrix_*(W_(currentQuat).transpose());


	dart::math::AngularJacobian angularJac = eePtr_->getAngularJacobian();
	dart::math::AngularJacobian angularJac_d = eePtr_->getAngularJacobianDeriv();
	Eigen::VectorXd dq = robotPtr_->getVelocities();

	Eigen::MatrixXd newJacobian = 0.5*block_one*angularJac;
	Eigen::Quaterniond tempQuatError = getErrorQuaternion();
	Eigen::VectorXd error_quat = quatVector(tempQuatError);
	/*
	error_quat(0) = getErrorQuaternion().w();
	error_quat(1) = getErrorQuaternion().x();
	error_quat(2) = getErrorQuaternion().y();
	error_quat(3) = getErrorQuaternion().z();
	*/
	std::cout<<"The orientation task erros is: "<<error_quat<<std::endl;

	Eigen::MatrixXd constant = 0.5*block_one*(angularJac_d + Kv_*angularJac)*dq + Kp_*error_quat;

	newJacobian = quatScalarWeightingMatrix_*newJacobian;
	constant = quatScalarWeightingMatrix_*constant;

	objQ_ = taskWeight_*newJacobian.transpose()*newJacobian;
	objP_ = taskWeight_*2*constant.transpose()*newJacobian;
	double tempC = 0;
	tempC = (constant.transpose()*constant)(0);
	objC_ = taskWeight_*tempC;

}

Eigen::MatrixXd orientationTask::Q_(const Eigen::Quaterniond &quaternion)const {

	Eigen::MatrixXd Q(4,4);
	Q.setZero();

	Q(0, 0) = quaternion.w();
	Q(1, 0) = quaternion.x();
	Q(2, 0) = quaternion.y();
	Q(3, 0) = quaternion.z();

	Q(0, 1) = -quaternion.x();
	Q(1, 1) = quaternion.w();
	Q(2, 1) = -quaternion.z();
	Q(3, 1) = quaternion.y();

	Q(0, 2) = -quaternion.y();
	Q(1, 2) = quaternion.z();
	Q(2, 2) = quaternion.w();
	Q(3, 2) = -quaternion.x();

	Q(0, 3) = -quaternion.z();
	Q(1, 3) = -quaternion.y();
	Q(2, 3) = quaternion.x();
	Q(3, 3) = quaternion.w();


	return Q;

}


Eigen::MatrixXd orientationTask::Q_bar_(const Eigen::Quaterniond &quaternion)const{

	Eigen::MatrixXd Q_bar(4,4);
	Q_bar.setIdentity();

	Q_bar(0, 0) = quaternion.w();
	Q_bar(1, 0) = -quaternion.x();
	Q_bar(2, 0) = -quaternion.y();
	Q_bar(3, 0) = -quaternion.z();

	Q_bar(0, 1) = quaternion.x();
	Q_bar(1, 1) = quaternion.w();
	Q_bar(2, 1) = quaternion.z();
	Q_bar(3, 1) = -quaternion.y();

	Q_bar(0, 2) = quaternion.y();
	Q_bar(1, 2) = -quaternion.z();
	Q_bar(2, 2) = quaternion.w();
	Q_bar(3, 2) = quaternion.x();

	Q_bar(0, 3) = quaternion.z();
	Q_bar(1, 3) = quaternion.y();
	Q_bar(2, 3) = -quaternion.x();
	Q_bar(3, 3) = quaternion.w();

	return Q_bar;
}


Eigen::MatrixXd orientationTask::W_(const Eigen::Quaterniond &quaternion) const{

	Eigen::MatrixXd quat_W(3,4);
	quat_W.setZero();

	quat_W(0, 0) = -quaternion.x();
	quat_W(1, 0) = -quaternion.y();
	quat_W(2, 0) = -quaternion.z();

	quat_W(0, 1) = quaternion.w();
	quat_W(1, 1) = quaternion.z();
	quat_W(2, 1) = -quaternion.y();

	quat_W(0, 2) = -quaternion.z();
	quat_W(1, 2) = quaternion.w();
	quat_W(2, 2) = quaternion.x();

	quat_W(0, 3) = quaternion.y();
	quat_W(1, 3) = -quaternion.x();
	quat_W(2, 3) = quaternion.w();

	return quat_W;


}




Eigen::MatrixXd orientationTask::W_prime_(const Eigen::Quaterniond &quaternion) const{

	Eigen::MatrixXd quat_W_prime(3,4);
	quat_W_prime.setZero();


	quat_W_prime(0, 0) = quaternion.x();
	quat_W_prime(1, 0) = quaternion.y();
	quat_W_prime(2, 0) = quaternion.z();

	quat_W_prime(0, 1) = quaternion.w();
	quat_W_prime(1, 1) = -quaternion.z();
	quat_W_prime(2, 1) = quaternion.y();

	quat_W_prime(0, 2) = quaternion.z();
	quat_W_prime(1, 2) = quaternion.w();
	quat_W_prime(2, 2) = -quaternion.x();

	quat_W_prime(0, 3) = -quaternion.y();
	quat_W_prime(1, 3) = quaternion.x();
	quat_W_prime(2, 3) = quaternion.w();

	return quat_W_prime;


}
