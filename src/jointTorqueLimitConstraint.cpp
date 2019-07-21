# include <manipulatorConstraints/jointTorqueLimitConstraint.hpp>


  jointTorqueLimitConstraint::jointTorqueLimitConstraint(const dart::dynamics::SkeletonPtr & skelPtr,
                             const int jointNumber,
                             bool lowerBoundIndicator,
                             const double bound)
  : dart::optimizer::Function(), robotPtr_(skelPtr), jointNumber_(jointNumber), lowerBoundIndicator_(lowerBoundIndicator), bound_(bound)
  {

    robotPtr_ = skelPtr;
    assert(robotPtr_ != nullptr);
    bound_ = bound;

    jointNumber_ = jointNumber;
    lowerBoundIndicator_ = lowerBoundIndicator;

    std::stringstream ss;
    if(lowerBoundIndicator_)
    {
      // bound_ = robotPtr_->getForceLowerLimit(jointNumber_);
      grad_ = Eigen::VectorXd(robotPtr_->getNumDofs());
      grad_.setZero();
      grad_(jointNumber_) = -1.0;
      ss << "Joint_" << jointNumber_ << "_" << robotPtr_->getJoint(jointNumber_ + 1)->getName()
         << "_torque_lower_limit_constraint";
      setName(ss.str());
    }
    else
    {
      // bound_ = robotPtr_->getForceUpperLimit(jointNumber_);
      grad_ = Eigen::VectorXd(robotPtr_->getNumDofs());
      grad_.setZero();
      grad_(jointNumber_) = 1.0;
      ss << "Joint_" << jointNumber_ << "_" << robotPtr_->getJoint(jointNumber_ + 1)->getName()
         << "_torque_upper_limit_constraint";
      setName(ss.str());
    }
  }


