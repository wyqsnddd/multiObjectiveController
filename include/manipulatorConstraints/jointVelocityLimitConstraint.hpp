/* Copyright 2018-2019 CNRS-UM LIRMM
 *
 * \author Yuquan Wang 
 *
 * 
 *
 * multiObjectiveController is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * pyQpController is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with multiObjectiveController. If not, see
 * <http://www.gnu.org/licenses/>.
 */

#ifndef JOINTVELOCITYLIMITCONSTRAINT_HPP
#define JOINTVELOCITYLIMITCONSTRAINT_HPP

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/optimizer/Function.hpp>
#include <vector>

class jointVelocityLimitConstraint : public dart::optimizer::Function
{
public:
  jointVelocityLimitConstraint(const dart::dynamics::SkeletonPtr & skelPtr,
                               const int jointNumber,
                               const bool lowerBoundIndicator)
  : dart::optimizer::Function()
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

  ~jointVelocityLimitConstraint() {}

  /**
   * Update the data that are used to calc the gradients and costs.
   */
  void update() {}

  virtual double eval(const Eigen::VectorXd & _x) override
  {
    if(lowerBoundIndicator_)
    {
      return -_x(jointNumber_) + (bound_ - robotPtr_->getVelocity(jointNumber_)) / robotPtr_->getTimeStep();
    }
    else
    {
      return _x(jointNumber_) - (bound_ - robotPtr_->getVelocity(jointNumber_)) / robotPtr_->getTimeStep();
    }
  }
  virtual void evalGradient(const Eigen::VectorXd & _x, Eigen::Map<Eigen::VectorXd> _grad) override
  {
    _grad = grad_;
  }

private:
  dart::dynamics::SkeletonPtr robotPtr_;

  /**
   * Index from zero
   */
  int jointNumber_;

  double bound_;
  bool lowerBoundIndicator_;

  Eigen::VectorXd grad_;
};

#endif
