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

#ifndef METAMANIPULATORTASK_HPP
#define METAMANIPULATORTASK_HPP

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <utils/utils.hpp>
#include <vector>

namespace pt = boost::property_tree;

class metaManipulatorTask
{
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  metaManipulatorTask(const std::string & endEffectorName,
                      const Eigen::Vector3d & goal,
                      const dart::dynamics::SkeletonPtr & skelPtr,
                      const pt::ptree configurationDataTree,
                      double weight,
                      double Kv,
                      double Kp,
                      Eigen::Vector3d selectionV)
  {

    robotPtr_ = skelPtr;
    assert(robotPtr_ != nullptr);
    // goal_ = goal;
    error_.setZero();

    // eePtr_ = robotPtr_->getEndEffector(endEffectorName);
    eePtr_ = robotPtr_->getBodyNode(endEffectorName);

    configurationDataTree_ = configurationDataTree;
    taskWeight_ = weight;
    Kp_ = Kp;
    Kv_ = Kv;
    selectionMatrix_.setZero();
    selectionMatrix_(0, 0) = selectionV(0);
    selectionMatrix_(1, 1) = selectionV(1);
    selectionMatrix_(2, 2) = selectionV(2);
    // BodyNodeIndex_ = BodyNodeIndex;
    setTarget(goal);

    objQ_.resize(robotPtr_->getNumDofs(), robotPtr_->getNumDofs());
    objQ_.setIdentity();
    objP_.resize(robotPtr_->getNumDofs());
    objP_.setZero();
    objC_ = 0.0;
  }
  ~metaManipulatorTask() {}
  virtual void update()
  {
    calcQ_();
    calcP_();
    calcC_();
  }

  Eigen::MatrixXd getQ() const
  {
    return objQ_;
  }
  Eigen::VectorXd getP() const
  {
    return objP_;
  }
  double getC() const
  {
    return objC_;
  }
  virtual void setTarget(Eigen::Vector3d newGoal)
  {
    goal_ = selectionMatrix_ * newGoal;
  }
  virtual Eigen::Vector3d getTarget() const
  {
    return goal_;
  }
  virtual Eigen::Vector3d getError() const
  {
    return error_;
  }
  virtual void calcError(const Eigen::Vector3d value)
  {
    error_ = selectionMatrix_ * (value - goal_);
  }

protected:
  double taskWeight_;
  double Kp_;
  double Kv_;
  Eigen::Matrix3d selectionMatrix_;
  // int BodyNodeIndex_;
  pt::ptree configurationDataTree_;

  dart::dynamics::SkeletonPtr robotPtr_;

  Eigen::MatrixXd objQ_;
  Eigen::VectorXd objP_;
  double objC_;

  Eigen::Vector3d goal_;
  Eigen::Vector3d error_;
  dart::dynamics::BodyNode * eePtr_;

  virtual void calcQ_() {}
  virtual void calcP_() {}
  virtual void calcC_() {}
};

#endif
