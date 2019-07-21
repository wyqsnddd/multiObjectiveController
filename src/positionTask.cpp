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

#include <manipulatorTasks/positionTask.hpp>
void positionTask::update()
{

  dart::math::LinearJacobian Jv = selectionMatrix_ * eePtr_->getLinearJacobian(); // 3 x n
  dart::math::LinearJacobian dJv = selectionMatrix_ * eePtr_->getLinearJacobianDeriv(); // 3 x n

  Eigen::Vector3d x = eePtr_->getTransform().translation();
  Eigen::VectorXd dq = robotPtr_->getVelocities(); // n x 1

  calcError(x);
  Eigen::Vector3d error = getError();

  //std::cout << "The position error is: " << error.transpose() << std::endl;

  Eigen::Vector3d tempConstant = (dJv + Kv_ * Jv) * dq + Kp_ * error;

  objQ_ = taskWeight_ * Jv.transpose() * Jv;
  objP_ = taskWeight_ * 2 * tempConstant.transpose() * Jv;
  objC_ = taskWeight_ * tempConstant.transpose() * tempConstant;

  // std::cout<<"The position Q is: "<< objQ_ <<std::endl;
  // std::cout<<"The position P is: "<< objP_<<std::endl;
  // std::cout<<"The position C is: "<< objC_<<std::endl;
}
