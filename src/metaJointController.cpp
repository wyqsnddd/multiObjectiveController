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

#include <jointControllers/metaJointController.hpp>

Eigen::VectorXd metaJointController::computedTorque_(const Eigen::VectorXd & acc)
{

  int nDof = static_cast<int>(robotPtr_->getNumDofs());
  Eigen::VectorXd tau(nDof);

  Eigen::VectorXd q = robotPtr_->getPositions();
  Eigen::VectorXd dq = robotPtr_->getVelocities();

  Eigen::VectorXd predict_dq = dq + dt_ * acc;
  Eigen::VectorXd predict_q = q + dt_ * predict_dq;

  Eigen::VectorXd q_error = q - predict_q;
  Eigen::VectorXd dq_error = dq - predict_dq;

  tau = robotPtr_->getMassMatrix() * (acc - Kv_ * dq_error - Kp_ * q_error) + robotPtr_->getCoriolisAndGravityForces();

  return tau;
}
