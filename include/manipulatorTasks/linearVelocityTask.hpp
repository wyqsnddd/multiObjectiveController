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

#ifndef LINEARVELOCITYTASK_HPP
#define LINEARVELOCITYTASK_HPP

#include <manipulatorTasks/metaManipulatorTask.hpp>

class linearVelocityTask : public metaManipulatorTask
{

public:
  linearVelocityTask(const std::string & endEffectorName,
                     const Eigen::Vector3d & desiredVelocity,
                     const dart::dynamics::SkeletonPtr & skelPtr,
                     const pt::ptree configurationDataTree,
                     double weight,
                     double Kv,
                     double Kp,
                     Eigen::Vector3d selectionV)
  : metaManipulatorTask(endEffectorName, desiredVelocity, skelPtr, configurationDataTree, weight, Kv, Kp, selectionV)
  {

    // std::cout<<"the initial translation is: "<< eePtr_->getTransform().translation()<<std::endl;
  }

  // linearVelocityTask(const std::string &endEffectorName, const Eigen::Vector3d & desiredVelocity, const
  // dart::dynamics::SkeletonPtr& skelPtr, const pt::ptree configurationDataTree, double weight, double Kv, double Kp,
  // Eigen::Vector3d selectionV):metaManipulatorTask(endEffectorName, desiredVelocity, skelPtr, configurationDataTree,
  // weight, Kv, Kp, selectionV ){

  //}
  void update() override;
  ~linearVelocityTask() {}
};

#endif
