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

#include <controllers/manipulatorQpController.hpp>

const std::shared_ptr<metaManipulatorTask> & manipulatorQpController::getTask(const std::string & taskName)
{
  auto tempEe = objPtr_->getTasks().find(taskName);
  if(tempEe != objPtr_->getTasks().end())
    return tempEe->second;
  else
  {
    // std::cout << "Link " << eeName << " is missing." << std::endl;
    std::string error_msg = std::string("manipulatorQpController::findTask: task ") + taskName + std::string(": does not exist.");
    throw std::runtime_error(error_msg);
  }
}
void manipulatorQpController::initializeOptimizer_(int dofs)
{

  probPtr_ = std::make_shared<dart::optimizer::Problem>(dofs);
  // objPtr_ = std::make_shared<sampleObjFunction>(this);
  objPtr_ = std::make_shared<manipulatorQpObj>(robotPtr_, configurationDataTree_);

  probPtr_->setObjective(objPtr_);
  // https://nlopt.readthedocs.io/en/latest/NLopt_Algorithms/
  // solverPtr_ = std::make_shared<dart::optimizer::NloptSolver>(probPtr_, nlopt::LD_SLSQP);
  // solverPtr_ = std::make_shared<dart::optimizer::NloptSolver>(probPtr_, nlopt::LD_LBFGS);
  // Working solvers:
  // solverPtr_ = std::make_shared<dart::optimizer::NloptSolver>(probPtr_, nlopt::LD_MMA);
  solverPtr_ = std::make_shared<dart::optimizer::NloptSolver>(probPtr_, nlopt::LD_CCSAQ);
  // solverPtr_ = std::make_shared<dart::optimizer::IpoptSolver>(probPtr_);

  solverOptimalityStatus_ = true;

  // Go through the joint position inequalities:
  for(unsigned ii = 0; ii < robotPtr_->getNumDofs(); ii++)
  {
    // add the upper bound for joint ii
    std::shared_ptr<jointPositionLimitConstraint> tempJointUpperPositionConstraintPtr =
        std::make_shared<jointPositionLimitConstraint>(robotPtr_, ii, false);
    inequalities_.push_back(tempJointUpperPositionConstraintPtr);
    probPtr_->addIneqConstraint(tempJointUpperPositionConstraintPtr);
    // add the lower bound for joint ii
    std::shared_ptr<jointPositionLimitConstraint> tempJointLowerPositionConstraintPtr =
        std::make_shared<jointPositionLimitConstraint>(robotPtr_, ii, true);
    inequalities_.push_back(tempJointLowerPositionConstraintPtr);
    probPtr_->addIneqConstraint(tempJointLowerPositionConstraintPtr);
  }

  // Go through the joint velocity inequalities:
  for(unsigned ii = 0; ii < robotPtr_->getNumDofs(); ii++)
  {
    std::shared_ptr<jointVelocityLimitConstraint> tempJointUpperVelocityConstraintPtr =
        std::make_shared<jointVelocityLimitConstraint>(robotPtr_, ii, false);
    inequalities_.push_back(tempJointUpperVelocityConstraintPtr);
    probPtr_->addIneqConstraint(tempJointUpperVelocityConstraintPtr);
    // add the lower bound for joint ii
    std::shared_ptr<jointVelocityLimitConstraint> tempJointLowerVelocityConstraintPtr =
        std::make_shared<jointVelocityLimitConstraint>(robotPtr_, ii, true);
    inequalities_.push_back(tempJointLowerVelocityConstraintPtr);
    probPtr_->addIneqConstraint(tempJointLowerVelocityConstraintPtr);
  }

  // Go through the joint acceleration inequalities:
  // By default, the joint acceleration limit is not specified
  for(unsigned ii = 0; ii < robotPtr_->getNumDofs(); ii++)
  {
    // add the upper bound for joint ii
    // double upperBound = configurationDataTree_.get<double>("qpController.accelerationLimits.upper", ii);
    double upperBound = as_vector<double>(configurationDataTree_, "qpController.accelerationLimits.upper")[ii];
    std::shared_ptr<jointAccelerationLimitConstraint> tempJointUpperAccelerationConstraintPtr =
        std::make_shared<jointAccelerationLimitConstraint>(robotPtr_, ii, false, upperBound);
    inequalities_.push_back(tempJointUpperAccelerationConstraintPtr);
    probPtr_->addIneqConstraint(tempJointUpperAccelerationConstraintPtr);

    // add the lower bound for joint ii
    double lowerBound = as_vector<double>(configurationDataTree_, "qpController.accelerationLimits.lower")[ii];
    std::shared_ptr<jointAccelerationLimitConstraint> tempJointLowerAccelerationConstraintPtr =
        std::make_shared<jointAccelerationLimitConstraint>(robotPtr_, ii, true, lowerBound);
    inequalities_.push_back(tempJointLowerAccelerationConstraintPtr);
    probPtr_->addIneqConstraint(tempJointLowerAccelerationConstraintPtr);
  }
  // Go through the torque limits
  for(unsigned ii = 0; ii < robotPtr_->getNumDofs(); ii++)
  {
    // add the upper bound for joint ii
    // double upperBound = configurationDataTree_.get<double>("qpController.accelerationLimits.upper", ii);
    double upperBound = as_vector<double>(configurationDataTree_, "qpController.torqueLimits.upper")[ii];
    std::shared_ptr<jointTorqueLimitConstraint> tempJointUpperTorqueConstraintPtr =
        std::make_shared<jointTorqueLimitConstraint>(robotPtr_, ii, false, upperBound);
    inequalities_.push_back(tempJointUpperTorqueConstraintPtr);
    probPtr_->addIneqConstraint(tempJointUpperTorqueConstraintPtr);

    // add the lower bound for joint ii
    double lowerBound = as_vector<double>(configurationDataTree_, "qpController.torqueLimits.lower")[ii];
    std::shared_ptr<jointTorqueLimitConstraint> tempJointLowerTorqueConstraintPtr =
        std::make_shared<jointTorqueLimitConstraint>(robotPtr_, ii, true, lowerBound);
    inequalities_.push_back(tempJointLowerTorqueConstraintPtr);
    probPtr_->addIneqConstraint(tempJointLowerTorqueConstraintPtr);
  }

} // end of optimizer initialization

const Eigen::VectorXd &  manipulatorQpController::readOptimalAcc()
{
  if(solverOptimalityStatus_)
    return optimalJointAcceleration_;
  else
    throw std::invalid_argument("The joint accelerations are not optimal");
}

bool manipulatorQpController::update()
{
  // update the constraints
  /*
  for (int ii = 0; ii<inequalities_.size();ii++){
    inequalities_[ii]->update();
  }
  */
  // update the objective
  objPtr_->update();

  solverOptimalityStatus_ = solverPtr_->solve();
  //std::cout << "The optimality status is: " << solverOptimalityStatus_ << std::endl;

  if(solverOptimalityStatus_)
    optimalJointAcceleration_ = probPtr_->getOptimalSolution();
  else
    optimalJointAcceleration_.setZero();

  //std::cout << "The generated joint acceleration is: " << std::endl << optimalJointAcceleration_ << std::endl;
  jointControllerPtr_->update(optimalJointAcceleration_);
  return true;
}
