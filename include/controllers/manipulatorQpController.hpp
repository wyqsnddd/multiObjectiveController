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

#ifndef MANIPULATORQPCONTROLLER_HPP
#define MANIPULATORQPCONTROLLER_HPP

#include "metaController.hpp"
#include <Eigen/Dense>
#include <dart/dart.hpp>
#include <dart/optimizer/Function.hpp>
#include <dart/optimizer/Problem.hpp>
#include <dart/optimizer/ipopt/IpoptSolver.hpp>
#include <dart/optimizer/nlopt/NloptSolver.hpp>
#include <manipulatorConstraints/jointAccelerationLimitConstraint.hpp>
#include <manipulatorConstraints/jointPositionLimitConstraint.hpp>
#include <manipulatorConstraints/jointTorqueLimitConstraint.hpp>
#include <manipulatorConstraints/jointVelocityLimitConstraint.hpp>
#include <manipulatorTasks/manipulatorQpObj.hpp>
#include <utils/utils.hpp>
#include <vector>

class manipulatorQpController : public metaController
{

public:
  manipulatorQpController(const dart::dynamics::SkeletonPtr & skelPtr, const pt::ptree configurationDataTree)
  : metaController(skelPtr, configurationDataTree)
  {

    optimalJointAcceleration_.setZero();

    initializeOptimizer_(static_cast<int>(robotPtr_->getNumDofs()));
  }
  ~manipulatorQpController() {}

  bool update() override;
  const Eigen::VectorXd & readOptimalAcc();
  /*
  const Eigen::VectorXd readP() const{
    return objP_;
  }
  const Eigen::MatrixXd readQ() const{
    return objQ_;
  }
  */
  int getRobotDof() const
  {
    return static_cast<int>(robotPtr_->getNumDofs());
  }

private:
  void initializeOptimizer_(int dofs);
  std::shared_ptr<dart::optimizer::Problem> probPtr_;
  std::shared_ptr<manipulatorQpObj> objPtr_;

  // Vector of inequality constraints
  std::vector<std::shared_ptr<dart::optimizer::Function>> inequalities_;

  // std::shared_ptr<dart::optimizer::IpoptSolver> solverPtr_;
  std::shared_ptr<dart::optimizer::NloptSolver> solverPtr_;
  bool solverOptimalityStatus_;

  //	double sampleCostFunction_(const Eigen::VectorXd& input);
  // Eigen::VectorXd objP_;
  // Eigen::MatrixXd objQ_;
  Eigen::VectorXd optimalJointAcceleration_;

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
};

/*
class sampleObjFunction: public dart::optimizer::Function {
  public:
    sampleObjFunction(const manipulatorQpController * qpPtr ): dart::optimizer::Function() {
      qpPtr_ = qpPtr;
    }
    virtual ~sampleObjFunction(){}
    double eval(const Eigen::VectorXd & _x) const override{
      double temp_1 = _x.transpose()*qpPtr_->readQ()*_x;
      double temp_2 = qpPtr_->readP().transpose()*_x;
      return temp_1 + temp_2;
    }
    void evalGradient(const Eigen::VectorXd & _x,
        Eigen::Map<Eigen::VectorXd> _grad) const override{
      _grad =  qpPtr_->readQ()*_x + qpPtr_->readP();

    }
    void evalHessian(const Eigen::VectorXd & _x,
        Eigen::Map<Eigen::VectorXd, Eigen::RowMajor> Hess) const override{
      Hess = qpPtr_->readQ();

    }
    void update(){};
  private:
    const manipulatorQpController * qpPtr_;
};
class sampleConstraintFunction: public dart::optimizer::Function {
  public:
    sampleConstraintFunction(const manipulatorQpController * qpPtr ): dart::optimizer::Function() {
      qpPtr_ = qpPtr;
    }
    virtual ~sampleConstraintFunction(){}
    double eval(const Eigen::VectorXd & _x) const override{
      double temp_1 = _x.transpose()*qpPtr_->readQ()*_x;
      double temp_2 = qpPtr_->readP().transpose()*_x;
      return temp_1 + temp_2;
    }
    void evalGradient(const Eigen::VectorXd & _x,
        Eigen::Map<Eigen::VectorXd> _grad) const override{
      _grad =  qpPtr_->readQ()*_x + qpPtr_->readP();

    }
    void evalHessian(const Eigen::VectorXd & _x,
        Eigen::Map<Eigen::VectorXd, Eigen::RowMajor> Hess) const override{
      // Hess = qpPtr_->readQ();

    }
  private:
    const manipulatorQpController * qpPtr_;
};



*/

#endif
