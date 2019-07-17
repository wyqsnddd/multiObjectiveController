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

#include <manipulatorTasks/manipulatorQpObj.hpp>

void manipulatorQpObj::update()
{

  initializeObjMatricies_();

  for(unsigned ii = 0; ii < tasks_.size(); ii++)
  {
    // for(int ii = 0; ii<2; ii++){
    std::cout << "Update task: " << ii << std::endl;

    tasks_[ii]->update();
    // Sum the Q matrix
    addToQ_(tasks_[ii]->getQ());
    // Sum the P matrix
    addToP_(tasks_[ii]->getP());
    // Sum teh C matrix
    addToC_(tasks_[ii]->getC());
  }
  // tasks_[2]->update();
  // 	std::cout<<"The qpObj: Q is: "<< objQ_ <<std::endl;
  // std::cout<<"The qpObj: P is: "<< objP_<<std::endl;
  // std::cout<<"The qpObj: C is: "<< objC_<<std::endl;
}

double manipulatorQpObj::eval(const Eigen::VectorXd & _x)
{
  double temp_one = _x.transpose() * readQ_() * _x;
  double temp_two = readP_().transpose() * _x;
  return temp_one + temp_two + readC_();
}
void manipulatorQpObj::initializeTasks_()
{
  // Read the json configuration file and add the tasks one by one

  // Initialize the joint weighting Q matrix
  jointWeights_Q_.resize(getDof_(), getDof_());
  jointWeights_Q_.setIdentity();
  double jointUnitWeight = configurationDataTree_.get<double>("qpController.jointUnitWeight", 0);
  jointWeights_Q_ = jointUnitWeight * jointWeights_Q_;

  // Initilize the position task:
  bool enablePositionTask = configurationDataTree_.get<bool>("qpController.positionTask.enabled", 0);
  if(enablePositionTask)
  {
    std::string eeName = configurationDataTree_.get<std::string>("qpController.positionTask.endEffectorName");
    double weight = configurationDataTree_.get<double>("qpController.positionTask.taskWeight", 0);

    double kv = configurationDataTree_.get<double>("qpController.positionTask.Kd", 0);
    double kp = configurationDataTree_.get<double>("qpController.positionTask.Kp", 0);

    Eigen::Vector3d desiredPosition;
    desiredPosition.setZero();
    Eigen::Vector3d selectionVector;
    selectionVector.setZero();

    for(int ii = 0; ii < 3; ii++)
    {
      desiredPosition(ii) = as_vector<double>(configurationDataTree_, "qpController.positionTask.setPoint")[ii];
      selectionVector(ii) = as_vector<double>(configurationDataTree_, "qpController.positionTask.axis_selection")[ii];
    }
    std::cout << "Position task weight: " << weight << ", eename: " << eeName << ", kv: " << kv << ", kp: " << kp
              << ", desiredPosition: " << desiredPosition.transpose() << ", selection vector: " << selectionVector
              << std::endl;
    std::shared_ptr<positionTask> positionTaskPtr = std::make_shared<positionTask>(
        eeName, desiredPosition, robotPtr_, configurationDataTree_, weight, kv, kp, selectionVector);

    tasks_.push_back(positionTaskPtr);
  } // end of position task initialization

  // Initilize the linear velocity task:
  bool enableLinearVelocityTask = configurationDataTree_.get<bool>("qpController.velocityTask.enabled", 0);
  if(enableLinearVelocityTask)
  {
    std::string eeName = configurationDataTree_.get<std::string>("qpController.velocityTask.endEffectorName");
    double weight = configurationDataTree_.get<double>("qpController.velocityTask.taskWeight", 0);

    double kv = configurationDataTree_.get<double>("qpController.velocityTask.Kd", 0);
    double kp = configurationDataTree_.get<double>("qpController.velocityTask.Kp", 0);

    Eigen::Vector3d desiredVelocity;
    desiredVelocity.setZero();
    Eigen::Vector3d selectionVector;
    selectionVector.setZero();

    for(int ii = 0; ii < 3; ii++)
    {
      desiredVelocity(ii) = as_vector<double>(configurationDataTree_, "qpController.velocityTask.desiredVelocity")[ii];
      selectionVector(ii) = as_vector<double>(configurationDataTree_, "qpController.velocityTask.axis_selection")[ii];
    }
    std::cout << "Linear velocity task weight: " << weight << ", eename: " << eeName << ", kv: " << kv << ", kp: " << kp
              << ", desiredVelocity: " << desiredVelocity.transpose() << ", selection vector: " << selectionVector
              << std::endl;
    std::shared_ptr<linearVelocityTask> linearVelocityTaskPtr = std::make_shared<linearVelocityTask>(
        eeName, desiredVelocity, robotPtr_, configurationDataTree_, weight, kv, kp, selectionVector);
    tasks_.push_back(linearVelocityTaskPtr);

  } // end of linear velocity task initialization

  // Initialize the orientation task:
  bool enableOrientationTask = configurationDataTree_.get<bool>("qpController.orientationTask.enabled", 0);
  if(enableOrientationTask)
  {
    std::string eeName = configurationDataTree_.get<std::string>("qpController.orientationTask.endEffectorName");
    double weight = configurationDataTree_.get<double>("qpController.orientationTask.taskWeight", 0);

    double kv = configurationDataTree_.get<double>("qpController.orientationTask.Kd", 0);
    double kp = configurationDataTree_.get<double>("qpController.orientationTask.Kp", 0);

    Eigen::Vector3d desiredVelocity;
    desiredVelocity.setZero();
    Eigen::Vector3d selectionVector;
    selectionVector.setZero();

    for(int ii = 0; ii < 3; ii++)
    {
      // desiredVelocity(ii) = as_vector<double>(configurationDataTree_,
      // "qpController.velocityTask.desiredVelocity")[ii];
      selectionVector(ii) =
          as_vector<double>(configurationDataTree_, "qpController.orientationTask.axis_selection")[ii];
    }

    bool sitStill = configurationDataTree_.get<bool>("qpController.orientationTask.stayStill", 0);
    Eigen::Quaterniond desiredQuaternion;
    desiredQuaternion.setIdentity();
    dart::dynamics::BodyNode * tempEePtr = robotPtr_->getBodyNode(eeName);
    if(sitStill)
    {
      Eigen::Quaterniond tempQuat(tempEePtr->getTransform().rotation());
      desiredQuaternion = tempQuat;
    }
    else
    {
      dart::dynamics::BodyNode * tempEePtr = robotPtr_->getBodyNode(eeName);

      double eulerX(0.0), eulerY(0.0), eulerZ(0.0);
      Eigen::Quaterniond currentQuat = Eigen::Quaterniond(tempEePtr->getTransform().rotation());

      eulerX = (as_vector<double>(configurationDataTree_, "qpController.orientationTask.setPoint-EulerAngle")[0]) * M_PI
               / 180.0;
      eulerY = (as_vector<double>(configurationDataTree_, "qpController.orientationTask.setPoint-EulerAngle")[1]) * M_PI
               / 180.0;
      eulerZ = (as_vector<double>(configurationDataTree_, "qpController.orientationTask.setPoint-EulerAngle")[2]) * M_PI
               / 180.0;

      std::cout << "The initial x is: " << eulerX << ", the initial Y is: " << eulerY
                << ", the initial z is: " << eulerZ << std::endl;

      desiredQuaternion =
          currentQuat
          * (Eigen::AngleAxisd(eulerZ, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(eulerY, Eigen::Vector3d::UnitY())
             * Eigen::AngleAxisd(eulerX, Eigen::Vector3d::UnitX()));
    }

    std::cout << "Orientation task weight: " << weight << ", eename: " << eeName << ", kv: " << kv << ", kp: " << kp
              << ", desiredOrientation: " << desiredQuaternion.w() << ", " << desiredQuaternion.vec()
              << ", selection vector: " << selectionVector << std::endl;
    std::shared_ptr<orientationTask> orientationTaskPtr = std::make_shared<orientationTask>(
        eeName, desiredQuaternion, robotPtr_, configurationDataTree_, weight, kv, kp, selectionVector);
    tasks_.push_back(orientationTaskPtr);

  } // end of orientation task initialization.
  // Initilize the collision avoidance task:
  bool enableCollisionAvoidanceTask = configurationDataTree_.get<bool>("qpController.collisionAvoidanceTask.enabled", 0);
  if(enableCollisionAvoidanceTask) 
  {
    double weight = configurationDataTree_.get<double>("qpController.collisionAvoidanceTask.taskWeight", 0);
    double kv = configurationDataTree_.get<double>("qpController.collisionAvoidanceTask.Kd", 0);
    double kp = configurationDataTree_.get<double>("qpController.collisionAvoidanceTask.Kp", 0);
  std::shared_ptr<collisionAvoidanceTask> collisionAvoidanceTaskPtr = std::make_shared<collisionAvoidanceTask>(
        robotPtr_, configurationDataTree_, weight, kv, kp);

    tasks_.push_back(collisionAvoidanceTaskPtr);

  }// end of collision avoidance task initialization.
}

void manipulatorQpObj::evalGradient(const Eigen::VectorXd & _x, Eigen::Map<Eigen::VectorXd> _grad) 
{
  _grad = readQ_() * _x + readP_();
}
