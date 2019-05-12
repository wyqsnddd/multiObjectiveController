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

#ifndef METAJOINTCONTROLLER_HPP
#define METAJOINTCONTROLLER_HPP

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <vector>

#ifndef pt_namespace
#  define pt_namespace
namespace pt = boost::property_tree;
#endif

class metaJointController
{

public:
  metaJointController(const dart::dynamics::SkeletonPtr & skelPtr, const pt::ptree configurationDataTree)
  {
    robotPtr_ = skelPtr;
    assert(robotPtr_ != nullptr);
    dt_ = robotPtr_->getTimeStep();

    configurationDataTree_ = configurationDataTree;
    Kv_ = configurationDataTree_.get<double>("jointControlConfiguration.computedTorqueControl.K_v", 0);
    Kp_ = configurationDataTree_.get<double>("jointControlConfiguration.computedTorqueControl.K_p", 0);
  }
  ~metaJointController() {}

  virtual bool update(const Eigen::VectorXd & inputAcc)
  {
    robotPtr_->setForces(computedTorque_(inputAcc));
    return true;
  }

protected:
  Eigen::VectorXd computedTorque_(const Eigen::VectorXd & acc);

  double dt_;
  dart::dynamics::SkeletonPtr robotPtr_;
  pt::ptree configurationDataTree_;

  double Kv_;
  double Kp_;
};

#endif
