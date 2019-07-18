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
# pragma once 

# include <manipulatorTasks/metaManipulatorTask.hpp>
# include <dart/collision/CollisionGroup.hpp> 
#include <dart/collision/DistanceOption.hpp>

class collisionAvoidanceTask: public metaManipulatorTask
{


 public: 
  collisionAvoidanceTask(
               const dart::dynamics::SkeletonPtr & skelPtr,
               const pt::ptree configurationDataTree,
               double weight,
               double Kv,
               double Kp
	       )
  : metaManipulatorTask(skelPtr, configurationDataTree, weight, Kv, Kp){
  
    resultPtr_ = new dart::collision::DistanceResult();
    std::cout << "The collision avoidance task is created "<<std::endl; 
    //initializeCollisionGroups(robotPtr_);
  }
   void update() override;
   void initializeCollisionGroups(
		   const dart::simulation::WorldPtr& world,
		   const dart::dynamics::SkeletonPtr & obstacleSkelPtr);
   void addObstacle(const dart::dynamics::SkeletonPtr & obstacleSkelPtr);
  ~collisionAvoidanceTask() {}

  // Collision group of the robot and the other collision group
  std::unique_ptr< dart::collision::CollisionGroup > robotCollisionGroupPtr_;
  std::unique_ptr< dart::collision::CollisionGroup > otherCollisionGroupPtr_;
  inline bool initialized()
  {
    return collisionGroupsInitialized_; 
  }
  inline const dart::collision::DistanceResult * getDistanceResult()
  {
    return resultPtr_; 
  }
 private:
  bool collisionGroupsInitialized_ = false;

  dart::collision::DistanceResult * resultPtr_;
};
