# ifndef JOINTACCELERATIONLIMITCONSTRAINT_HPP
# define JOINTACCELERATIONLIMITCONSTRAINT_HPP

# include <vector>
# include <Eigen/Dense>

# include <dart/dart.hpp>
# include <dart/optimizer/Function.hpp>


class jointAccelerationLimitConstraint : public dart::optimizer::Function {
	public:
		jointAccelerationLimitConstraint (const dart::dynamics::SkeletonPtr& skelPtr, const int jointNumber, const bool lowerBoundIndicator , const double bound): dart::optimizer::Function(){

			robotPtr_ = skelPtr;
			assert(robotPtr_ != nullptr);
			bound_ = bound;

			jointNumber_ = jointNumber;
			lowerBoundIndicator_ = lowerBoundIndicator;

			std::stringstream ss;
			if(lowerBoundIndicator_){
				// bound_ = robotPtr_->getAccelerationLowerLimit(jointNumber_); 
				grad_ = Eigen::VectorXd(robotPtr_->getNumDofs());
				grad_.setZero();
				grad_(jointNumber_) = -1.0;

				ss<<"Joint_"<<jointNumber_<<"_"<<robotPtr_->getJoint(jointNumber_+1)->getName()<<"_acceleration_lower_limit_constraint";
				setName(ss.str());
			}else{
				// bound_ = robotPtr_->getAccelerationUpperLimit(jointNumber_); 
				grad_ = Eigen::VectorXd(robotPtr_->getNumDofs());
				grad_.setZero();
				grad_(jointNumber_) = 1.0;
				ss<<"Joint_"<<jointNumber_<<"_"<<robotPtr_->getJoint(jointNumber_+1)->getName()<<"_acceleration_upper_limit_constraint";
				setName(ss.str());

			}

			std::cout<<getName()<<" bound is: "<<bound_<<std::endl;
		}

		~jointAccelerationLimitConstraint(){
		}

		/**
		 * Update the data that are used to calc the gradients and costs.
		 */
		void update(){
		}

		double eval(const Eigen::VectorXd& _x ) const override{
			if(lowerBoundIndicator_){
				return - _x(jointNumber_) +  bound_;
			}	else{
				return  _x(jointNumber_) -  bound_;
			}
		}
		void evalGradient(const Eigen::VectorXd& _x, Eigen::Map<Eigen::VectorXd> _grad ) const override{
			_grad = grad_;
		}
	private:

	dart::dynamics::SkeletonPtr robotPtr_;
	
	/** 
	 * Index from zero
	 */
	int  jointNumber_;

	double bound_;
	bool lowerBoundIndicator_;
	
	Eigen::VectorXd grad_;


};

# endif
