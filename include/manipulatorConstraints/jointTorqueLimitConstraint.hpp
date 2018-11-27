# ifndef JOINTTORQUELIMITCONSTRAINT_HPP
# define JOINTTORQUELIMITCONSTRAINT_HPP

# include <vector>
# include <Eigen/Dense>

# include <dart/dart.hpp>
# include <dart/optimizer/Function.hpp>


class jointTorqueLimitConstraint : public dart::optimizer::Function {
	public:
		jointTorqueLimitConstraint (const dart::dynamics::SkeletonPtr& skelPtr, const int jointNumber, bool lowerBoundIndicator, const double bound ): dart::optimizer::Function(){

			robotPtr_ = skelPtr;
			assert(robotPtr_ != nullptr);
			bound_ = bound;

			jointNumber_ = jointNumber;
			lowerBoundIndicator_ = lowerBoundIndicator;

			std::stringstream ss;
			if(lowerBoundIndicator_){
				// bound_ = robotPtr_->getForceLowerLimit(jointNumber_); 
				grad_ = Eigen::VectorXd(robotPtr_->getNumDofs());
				grad_.setZero();
				grad_(jointNumber_) = -1.0;
				ss<<"Joint_"<<jointNumber_<<"_"<<robotPtr_->getJoint(jointNumber_+1)->getName()<<"_torque_lower_limit_constraint";
				setName(ss.str());
			}else{
				// bound_ = robotPtr_->getForceUpperLimit(jointNumber_); 
				grad_ = Eigen::VectorXd(robotPtr_->getNumDofs());
				grad_.setZero();
				grad_(jointNumber_) = 1.0;
				ss<<"Joint_"<<jointNumber_<<"_"<<robotPtr_->getJoint(jointNumber_+1)->getName()<<"_torque_upper_limit_constraint";
				setName(ss.str());

			}
		}

		~jointTorqueLimitConstraint(){
		}

		/**
		 * Update the data that are used to calc the gradients and costs.
		 */
		void update(){
		}

		double eval(const Eigen::VectorXd& _x ) const override{
			Eigen::VectorXd Cg = robotPtr_->getCoriolisAndGravityForces();

			/* The reverse does not work*/
			/*
			Eigen::VectorXd boundVector(robotPtr_->getNumDofs());
			boundVector<< bound_ , bound_, bound_, bound_, bound_, bound_; 

			if(lowerBoundIndicator_){
				return -_x[jointNumber_]  + (robotPtr_->getInvMassMatrix()*(boundVector - robotPtr_->getCoriolisAndGravityForces() ))(jointNumber_);
			}else{
				return  _x[jointNumber_] - (robotPtr_->getInvMassMatrix()*(boundVector - robotPtr_->getCoriolisAndGravityForces() ))(jointNumber_);
			}
			*/
			if(lowerBoundIndicator_){
				return (- robotPtr_->getMassMatrix().block<1,6>(jointNumber_, 0)*_x)  + (bound_ - robotPtr_->getCoriolisAndGravityForces()[jointNumber_]);
			}else{
				return  (robotPtr_->getMassMatrix().block<1,6>(jointNumber_, 0)*_x) - (bound_ - robotPtr_->getCoriolisAndGravityForces()[jointNumber_]);
			}
		}
		void evalGradient(const Eigen::VectorXd& _x, Eigen::Map<Eigen::VectorXd> _grad ) const override{
			if(lowerBoundIndicator_){
				_grad = - robotPtr_->getMassMatrix().block<1,6>(jointNumber_, 0);
			}else{
				_grad = robotPtr_->getMassMatrix().block<1,6>(jointNumber_, 0);
			}
			//_grad = grad_;
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
