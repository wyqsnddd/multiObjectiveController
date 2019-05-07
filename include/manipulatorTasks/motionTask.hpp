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

#ifndef MOTIONTASK_HPP
#define MOTIONTASK_HPP

#include <dart/dynamics/SimpleFrame.hpp>
#include <manipulatorTasks/metaManipulatorTask.hpp>

class motionTask : public metaManipulatorTask
{

public:
  /// We use this task to track the motion of reference frame with the end-effector frame
  motionTask(const std::string & endEffectorName,
             const dart::dynamics::SkeletonPtr & skelPtr,
             const pt::ptree configurationDataTree,
             double weight,
             double Kv,
             double Kp,
             Eigen::Vector3d selectionV,
             std::shared_ptr<dart::dynamics::SimpleFrame> referenceFramePtr)
  : metaManipulatorTask(endEffectorName,
                        referenceFramePtr->getTransform()
                            .translation(), /// By default, it returns the transform w.r.t. the world frame.
                        skelPtr,
                        configurationDataTree,
                        weight,
                        Kv,
                        Kp,
                        selectionV)
  {
    std::cout << "the initial translation is: " << eePtr_->getTransform().translation() << std::endl;
    referenceFramePtr_ = referenceFramePtr;
    updateReferences_();
  }
  void update() override;
  ~motionTask() {}

private:
  void updateReferences_(); // Update the desired position, velocity and acceleration.
  std::shared_ptr<dart::dynamics::SimpleFrame> referenceFramePtr_; // Pointer to the reference frame
  Eigen::Vector3d velGoal_;
  Eigen::Vector3d accGoal_;
};

#endif
