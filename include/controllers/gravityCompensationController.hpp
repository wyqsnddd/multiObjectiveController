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

#ifndef GRAVITYCOMPENSATIONCONTROLLER_H
#define GRAVITYCOMPENSATIONCONTROLLER_H

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <vector>

class gravityCompensationController
{

public:
  gravityCompensationController(const dart::dynamics::SkeletonPtr & skelPtr)
  {
    enabled_ = true;
    robotPtr_ = skelPtr;
    gravity_.setZero();
    gravity_ = robotPtr_->getGravity();
  }

  ~gravityCompensationController() {}
  Eigen::VectorXd computeTorques();
  void update()
  {
    robotPtr_->setForces(computeTorques());
  }

private:
  bool enabled_;
  dart::dynamics::SkeletonPtr robotPtr_;
  Eigen::Vector3d gravity_;
};

#endif
