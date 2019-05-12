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

#ifndef METACONTROLLER_H
#define METACONTROLLER_H

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <jointControllers/metaJointController.hpp>
#include <vector>

#ifndef pt_namespace
#  define pt_namespace
namespace pt = boost::property_tree;
#endif

class metaController
{
public:
  metaController(const dart::dynamics::SkeletonPtr & skelPtr, const pt::ptree configurationDataTree);
  ~metaController() {}
  // virtual Eigen::VectorXd computeTorques();

  virtual bool update()
  {
    //	robotPtr_->setForces(computeTorques());
    Eigen::VectorXd tempAcc(robotPtr_->getNumDofs());
    tempAcc.setZero();
    jointControllerPtr_->update(tempAcc);
    // robotPtr_->setForces(accToTorque(tempAcc));
    return true;
  }

  // Eigen::VectorXd accToTorque(const Eigen::VectorXd &acc);

  int readControlMode() const
  {
    return controlMode_;
  }
  dart::dynamics::SkeletonPtr getRobot() const
  {
    return robotPtr_;
  }

protected:
  bool enabled_;
  dart::dynamics::SkeletonPtr robotPtr_;
  double dt_;

  pt::ptree configurationDataTree_;
  std::shared_ptr<metaJointController> jointControllerPtr_;

  int controlMode_;

  /**
   * We use the computed torque control to realize a desired joint acceleration.
   * @param acc The desired joint accleration.
   * @return The corresponding torque
   */
  // Eigen::VectorXd computedTorque_(const Eigen::VectorXd &acc);
  /**
   * We use the computed torque control to realize a desired joint acceleration.
   * @param acc The desired joint accleration.
   * @return The corresponding torque
   */
  // Eigen::VectorXd jointVelocityControl_(const Eigen::VectorXd &acc);
};
#endif
