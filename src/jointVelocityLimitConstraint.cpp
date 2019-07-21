# include <manipulatorConstraints/jointVelocityLimitConstraint.hpp>


 
jointVelocityLimitConstraint::jointVelocityLimitConstraint(const dart::dynamics::SkeletonPtr & skelPtr,
                               const int jointNumber,
                               const bool lowerBoundIndicator)
  : dart::optimizer::Function(), robotPtr_(skelPtr), jointNumber_(jointNumber), lowerBoundIndicator_(lowerBoundIndicator)
  {

    robotPtr_ = skelPtr;
    assert(robotPtr_ != nullptr);

    jointNumber_ = jointNumber;
    lowerBoundIndicator_ = lowerBoundIndicator;

    std::stringstream ss;
    if(lowerBoundIndicator_)
    {
      bound_ = robotPtr_->getVelocityLowerLimit(jointNumber_);
      grad_ = Eigen::VectorXd(robotPtr_->getNumDofs());
      grad_.setZero();
      grad_(jointNumber_) = -1.0;
      ss << "Joint_" << jointNumber_ << "_" << robotPtr_->getJoint(jointNumber_ + 1)->getName()
         << "_velocity_lower_limit_constraint";
      setName(ss.str());
    }
    else
    {
      bound_ = robotPtr_->getVelocityUpperLimit(jointNumber_);
      grad_ = Eigen::VectorXd(robotPtr_->getNumDofs());
      grad_.setZero();
      grad_(jointNumber_) = 1.0;
      ss << "Joint_" << jointNumber_ << "_" << robotPtr_->getJoint(jointNumber_ + 1)->getName()
         << "_velocity_upper_limit_constraint";
      setName(ss.str());
    }

    std::cout << getName() << " bound is: " << bound_ << std::endl;
  }


