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

#ifndef JOINTPOSITIONUPPERLIMITCONSTRAINT_HPP
#define JOINTPOSITIONUPPERLIMITCONSTRAINT_HPP

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/optimizer/Function.hpp>
#include <vector>

class jointPositionUpperLimitConstraint : public dart::optimizer::Function
{
public:
  jointPositionUpperLimitConstraintconst(const dart::dynamics::SkeletonPtr & skelPtr) : dart::optimizer::Function()
  {
    robotPtr_ = skelPtr;
    assert(robotPtr_ != nullptr);

    upperLimit_ = robotPtr_->getPositionUpperLimits();
      lowerLimit_ = robotPtr_->getPositionLowerLimits(
  }

  ~jointPositionUpperLimitConstraint() {}

  /**
   * Update the data that are used to calc the gradients and costs.
   */
  void update() {}

  double eval(const Eigen::VectorXd & _x) const override
  {
    return
  }
  double evalGradient(const Eigen::VectorXd & _x, Eigen::Map<Eigen::VectorXd> _grad) const override
  {
    return
  }

private:
  const dart::dynamics::SkeletonPtr robotPtr_;

  Eigen::VectorXd upperLimit_;
  Eigen::VectorXd lowerLimit_;

  Eigen::VectorXd & upperRHS_() const
  {
    return (upperLimit_ - robotPtr_->getPositions()) / pow(robotPtr_->getTimeStep(), 2)
           - robotPtr_->getVelocities() / robotPtr_->getTimeStep();
  }

  Eigen::VectorXd & lowerRHS_() const
  {
  return -(
			lowerLimit_ - robotPtr_->getPositions())/pow(robotPtr_->getTimeStep(), 2) - robotPtr_->getVelocities()/robotPtr_->getTimeStep()
		);
  }

  Eigen::MatrixXd & upperLHS_() const
  {
    return Eigen::MatrixXd::Identity(robotPtr_->getNumDofs());
  }
  Eigen::MatrixXd & lowerLHS_() const
  {
    return -Eigen::MatrixXd::Identity(robotPtr_->getNumDofs());
  }
};

#endif
