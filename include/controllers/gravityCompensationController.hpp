# ifndef GRAVITYCOMPENSATIONCONTROLLER_H
# define GRAVITYCOMPENSATIONCONTROLLER_H

#include <vector>

#include <Eigen/Dense>

#include <dart/dart.hpp>


class gravityCompensationController{

	public:
		gravityCompensationController( const dart::dynamics::SkeletonPtr& skelPtr
				){
			enabled_ = true;
			robotPtr_ = skelPtr;
			gravity_.setZero();
			gravity_ = robotPtr_->getGravity();
		}

		~gravityCompensationController(){}
		Eigen::VectorXd computeTorques();
		void update(){
			robotPtr_->setForces(computeTorques());
		}
	private:
	bool enabled_;	
	dart::dynamics::SkeletonPtr robotPtr_; 
	Eigen::Vector3d gravity_;
};

# endif
