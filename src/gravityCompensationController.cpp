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

#include <controllers/gravityCompensationController.hpp>
Eigen::VectorXd gravityCompensationController::computeTorques()
{
  int nDof = robotPtr_->getNumDofs();
  Eigen::VectorXd tau(nDof);
  // tau = Eigen::VectorXd::Zero(nDof);
  tau.setZero();

  if(!enabled_)
  {
    return tau;
  }
  else
  {

    Eigen::VectorXd Cg = robotPtr_->getCoriolisAndGravityForces();
    tau = Cg;
    /*
    for (int ii = 0; ii< robotPtr_->getNumBodyNodes(); ii++){
      dart::dynamics::BodyNode* tempBodyPtr = robotPtr_->getBodyNode(ii);
      double m = tempBodyPtr->getMass();
      if(tempBodyPtr->getNumDependentDofs () > 0){
        dart::math::LinearJacobian tempJacobian = tempBodyPtr->getLinearJacobian(tempBodyPtr->getLocalCOM());
        tau += -tempJacobian.transpose()*(m*gravity_);
      }
    }
    */
    // std::cout<<"The computed torque is: "<<tau<<std::endl;
    return tau;
  }
}
