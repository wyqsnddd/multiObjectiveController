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

#ifndef MANIPULATORQPOBJ_HPP
#define MANIPULATORQPOBJ_HPP

#include <boost/property_tree/json_parser.hpp>
#include <boost/property_tree/ptree.hpp>

#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Problem.hpp>
#include <vector>

//# include <controllers/manipulatorQpController.hpp>

#include <manipulatorTasks/linearVelocityTask.hpp>
#include <manipulatorTasks/metaManipulatorTask.hpp>
#include <manipulatorTasks/orientationTask.hpp>
#include <manipulatorTasks/positionTask.hpp>
#include <manipulatorTasks/collisionAvoidanceTask.hpp>
#include <utils/utils.hpp>

class manipulatorQpObj : public dart::optimizer::Function
{
public:
  manipulatorQpObj(const dart::dynamics::SkeletonPtr & skelPtr, const pt::ptree configurationDataTree)
  : dart::optimizer::Function()
  {

    // qpControllerPtr_ = qpControllerPtr;
    robotPtr_ = skelPtr;
    configurationDataTree_ = configurationDataTree;
    dof_ = static_cast<int>(robotPtr_->getNumDofs());

    initializeTasks_();
    initializeObjMatricies_();
  }

  ~manipulatorQpObj() {}
  virtual double eval(const Eigen::VectorXd & _x) override;
  virtual void evalGradient(const Eigen::VectorXd & _x, Eigen::Map<Eigen::VectorXd> _grad) override;
  void update();

private:
  // const manipulatorQpController * qpControllerPtr_;
  dart::dynamics::SkeletonPtr robotPtr_;
  pt::ptree configurationDataTree_;
  /*
     template <typename T>
     std::vector<T> as_vector_(pt::ptree const& ptIn, pt::ptree::key_type const& key)
     {
     std::vector<T> r;
     for (auto& item : ptIn.get_child(key))
     r.push_back(item.second.get_value<T>());
     return r;
     }
     */
  std::vector<std::shared_ptr<metaManipulatorTask>> tasks_;
  void initializeTasks_();

  Eigen::MatrixXd jointWeights_Q_;

  Eigen::MatrixXd objQ_;
  Eigen::VectorXd objP_;
  double objC_;
  int getDof_() const
  {
    return dof_;
  }
  int dof_;

  void initializeObjMatricies_()
  {

    objP_ = Eigen::VectorXd::Zero(getDof_());

    objQ_ = Eigen::MatrixXd::Zero(getDof_(), getDof_());
    addToQ_(jointWeights_Q_);

    objC_ = 0.0;
  }
  Eigen::MatrixXd readQ_() const
  {
    return objQ_;
  }
  Eigen::VectorXd readP_() const
  {
    return objP_;
  }
  double readC_() const
  {
    return objC_;
  }
  void addToQ_(const Eigen::MatrixXd & inputQ)
  {
    objQ_ = objQ_ + inputQ;
  }
  void addToP_(const Eigen::VectorXd & inputP)
  {
    objP_ = objP_ + inputP;
  }
  void addToC_(const double inputC)
  {
    objC_ = objC_ + inputC;
  }
};

#endif
