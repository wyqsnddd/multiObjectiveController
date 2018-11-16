# ifndef JOINTVELOCITYLIMITCONSTRAINT_HPP
# define JOINTVELOCITYLIMITCONSTRAINT_HPP

# include <vector>
# include <Eigen/Dense>

# include <dart/dart.hpp>
# include <dart/optimizer/Function.hpp>


class jointVelocityLimitConstraint : public dart::optimizer::Function {
	public:
		jointVelocityLimitConstraint (const dart::dynamics::SkeletonPtr& skelPtr, const int jointNumber, const bool lowerBoundIndicator ): dart::optimizer::Function(){

			robotPtr_ = skelPtr;
			assert(robotPtr_ != nullptr);

			jointNumber_ = jointNumber;
			lowerBoundIndicator_ = lowerBoundIndicator;

			std::stringstream ss;
			if(lowerBoundIndicator_){
				bound_ = robotPtr_->getVelocityLowerLimit(jointNumber_); 
				grad_ = Eigen::VectorXd(robotPtr_->getNumDofs());
				grad_.setZero();
				grad_(jointNumber_) = -1.0;
				ss<<"Joint_"<<jointNumber_<<"_"<<robotPtr_->getJoint(jointNumber_+1)->getName()<<"_velocity_lower_limit_constraint";
				setName(ss.str());

			}else{
				bound_ = robotPtr_->getVelocityUpperLimit(jointNumber_); 
				grad_ = Eigen::VectorXd(robotPtr_->getNumDofs());
				grad_.setZero();
				grad_(jointNumber_) = 1.0;
				ss<<"Joint_"<<jointNumber_<<"_"<<robotPtr_->getJoint(jointNumber_+1)->getName()<<"_velocity_upper_limit_constraint";
				setName(ss.str());


			}

			std::cout<<getName()<<" bound is: "<<bound_<<std::endl;
		}

		~jointVelocityLimitConstraint(){
		}

		/**
		 * Update the data that are used to calc the gradients and costs.
		 */
		void update(){
		}

		double eval(const Eigen::VectorXd& _x ) const override{
			if(lowerBoundIndicator_){
				return - _x(jointNumber_) + ( bound_ - robotPtr_->getVelocity(jointNumber_))/robotPtr_->getTimeStep();
			}	else{
				return  _x(jointNumber_) - ( bound_ - robotPtr_->getVelocity(jointNumber_))/robotPtr_->getTimeStep();
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
