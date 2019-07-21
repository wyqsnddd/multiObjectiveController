# include <manipulatorConstraints/jointAccelerationLimitConstraint.hpp>
jointAccelerationLimitConstraint::jointAccelerationLimitConstraint(const dart::dynamics::SkeletonPtr & skelPtr,
                                   const int jointNumber,
                                   const bool lowerBoundIndicator,
                                   const double bound)
  : dart::optimizer::Function(), robotPtr_(skelPtr), jointNumber_(jointNumber), lowerBoundIndicator_(lowerBoundIndicator), bound_(bound) 
  {

    bound_ = bound;

    jointNumber_ = jointNumber;
    lowerBoundIndicator_ = lowerBoundIndicator;

    std::stringstream ss;
    if(lowerBoundIndicator_)
    {
      // bound_ = robotPtr_->getAccelerationLowerLimit(jointNumber_);
      grad_ = Eigen::VectorXd(robotPtr_->getNumDofs());
      grad_.setZero();
      grad_(jointNumber_) = -1.0;

      ss << "Joint_" << jointNumber_ << "_" << robotPtr_->getJoint(jointNumber_ + 1)->getName()
         << "_acceleration_lower_limit_constraint";
      setName(ss.str());
    }
    else
    {
      // bound_ = robotPtr_->getAccelerationUpperLimit(jointNumber_);
      grad_ = Eigen::VectorXd(robotPtr_->getNumDofs());
      grad_.setZero();
      grad_(jointNumber_) = 1.0;
      ss << "Joint_" << jointNumber_ << "_" << robotPtr_->getJoint(jointNumber_ + 1)->getName()
         << "_acceleration_upper_limit_constraint";
      setName(ss.str());
    }

    std::cout << getName() << " bound is: " << bound_ << std::endl;
  }


