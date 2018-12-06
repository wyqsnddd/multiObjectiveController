# ifndef JOINTPOSITIONUPPERLIMITCONSTRAINT_HPP
# define JOINTPOSITIONUPPERLIMITCONSTRAINT_HPP

# include <vector>
# include <Eigen/Dense>

# include <dart/dart.hpp>
# include <dart/optimizer/Function.hpp>


class jointPositionUpperLimitConstraint : public dart::optimizer::Function {
	public:
		jointPositionUpperLimitConstraintconst (const dart::dynamics::SkeletonPtr& skelPtr ): dart::optimizer::Function(){
			robotPtr_ = skelPtr;
			assert(robotPtr_ != nullptr);

			upperLimit_ = robotPtr_->getPositionUpperLimits(); 
			lowerLimit_ = robotPtr_->getPositionLowerLimits(
		}

		~jointPositionUpperLimitConstraint(){
		}

		/**
		 * Update the data that are used to calc the gradients and costs.
		 */
		void update(){
		}

		double eval(const Eigen::VectorXd& _x ) const override{
			return 	
		}
		double evalGradient(const Eigen::VectorXd& _x, Eigen::Map<Eigen::VectorXd> _grad ) const override{
			return 	
		}
	private:

	const dart::dynamics::SkeletonPtr robotPtr_;

	Eigen::VectorXd upperLimit_;
	Eigen::VectorXd lowerLimit_;

	Eigen::VectorXd upperRHS_() const {
	return (upperLimit_ - robotPtr_->getPositions())/pow(robotPtr_->getTimeStep(), 2) - robotPtr_->getVelocities()/robotPtr_->getTimeStep();
	}

	Eigen::VectorXd lowerRHS_() const {
	return -(
			lowerLimit_ - robotPtr_->getPositions())/pow(robotPtr_->getTimeStep(), 2) - robotPtr_->getVelocities()/robotPtr_->getTimeStep()
		);
	}

	Eigen::MatrixXd upperLHS_() const {
		return Eigen::MatrixXd::Identity(robotPtr_->getNumDofs());
	}
	Eigen::MatrixXd lowerLHS_() const {
		return -Eigen::MatrixXd::Identity(robotPtr_->getNumDofs());
	}


};

# endif
