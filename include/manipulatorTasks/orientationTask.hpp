# ifndef ORIENTATIONTASK_HPP
# define ORIENTATIONTASK_HPP

# include <manipulatorTasks/metaManipulatorTask.hpp>

class orientationTask: public metaManipulatorTask{
	public: 
		orientationTask(const std::string &endEffectorName, const Eigen::Quaterniond& desiredOrientation, const dart::dynamics::SkeletonPtr& skelPtr, const pt::ptree configurationDataTree, double weight, double Kv, double Kp, Eigen::Vector3d selectionV):metaManipulatorTask(endEffectorName, desiredOrientation.vec(), skelPtr, configurationDataTree, weight, Kv, Kp, selectionV ){

			std::cout<<"the initial rotation is: "<< eePtr_->getTransform().rotation()<<std::endl;
			errorOrientation_.setIdentity();
			goalOrientation_.setIdentity();
			
			setQuaternion(desiredOrientation);

			deleteOperationMatrix_.resize(3,4);
			deleteOperationMatrix_.setZero();
			deleteOperationMatrix_.block<3,3>(0, 1) = Eigen::MatrixXd::Identity(3,3);
			std::cout<<"The initial deletion operation matrix is: "<<std::endl<<deleteOperationMatrix_<<std::endl;
			// conjugate_operation_Matrix_.resize(4,4);
			// conjugate_operation_Matrix_.setIdentity();
			// conjugate_operation_Matrix_ = - conjugate_operation_Matrix_;
			// conjugate_operation_Matrix_(0,0) = 1.0;
			// std::cout<<"The initial conjugate operation matrix is: "<<std::endl<<conjugate_operation_Matrix_<<std::endl;

			desiredQuatMatrix_.resize(4,4);
			desiredQuatMatrix_ = Q_bar_(desiredOrientation);
			std::cout<<"The desired quaternion is: "<<std::endl<<desiredOrientation.w()<<", "<<desiredOrientation.vec().transpose()<<std::endl;
			std::cout<<"The desired quat matrix is: "<<std::endl<<desiredQuatMatrix_<<std::endl;
			Eigen::Matrix3d currentRotation =  eePtr_->getTransform().rotation();
			Eigen::Quaterniond currentQuat = Eigen::Quaterniond(currentRotation);

			std::cout<<"The current quaternion is: "<<std::endl<<currentQuat.w()<<", "<<currentQuat.vec().transpose()<<std::endl;
			calcQuatError(currentQuat);

			std::cout<<"The quaternion error is: "<<std::endl<<getErrorQuaternion().w()<<", "<<getErrorQuaternion().vec().transpose()<<std::endl;
			
			// quatScalarWeightingMatrix_.resize(4,4);
			// quatScalarWeightingMatrix_.setIdentity();
			// quatScalarWeightingMatrix_(0, 0) = configurationDataTree_.get<double>("qpController.orientationTask.quaternion_scalar_weight", 0); 

			std::cout<<"The initial quat scalar matrix is: "<<std::endl<<quatScalarWeightingMatrix_<<std::endl;
		}

		void update() override;

		void setQuaternion(Eigen::Quaterniond newTarget){
			goalOrientation_ = newTarget;
		}
		Eigen::Quaterniond getQuaternion() const{
			return goalOrientation_;	
		}
		Eigen::Quaterniond getErrorQuaternion() const {
			return errorOrientation_;	
		}

                Eigen::Vector4d quatVector_(Eigen::Quaterniond& value)const{
		  Eigen::Vector4d temp;
		  temp.setZero();
		  temp(0) = value.w();
		  temp.tail(3) = value.vec();
		  return temp;
		}
  
		void calcQuatError(Eigen::Quaterniond& value){
			Eigen::Vector4d temp_vec = desiredQuatMatrix_*(quatVector_(value));
			errorOrientation_.w() = temp_vec(0);
			errorOrientation_.vec() = temp_vec.tail(3);

		}
		~orientationTask(){}
	protected:
		Eigen::Quaterniond goalOrientation_; 
		Eigen::Quaterniond errorOrientation_; 

		// Eigen::MatrixXd conjugate_operation_Matrix_;
		Eigen::MatrixXd deleteOperationMatrix_;

                Eigen::MatrixXd desiredQuatMatrix_;
		Eigen::MatrixXd quatScalarWeightingMatrix_;

		Eigen::MatrixXd Q_(const Eigen::Quaterniond &input)const;
		Eigen::MatrixXd Q_bar_(const Eigen::Quaterniond &input)const;

		Eigen::MatrixXd W_(const Eigen::Quaterniond &input) const;
		Eigen::MatrixXd W_prime_(const Eigen::Quaterniond &input) const;
};
# endif
