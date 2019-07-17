# include <manipulatorTasks/collisionAvoidanceTask.hpp>


void collisionAvoidanceTask::update()
{
  if(!initialized())
    throw std::runtime_error("collisionAvoidanceTask: the CollisionGroups have not yet initialized. ");
  dart::collision::DistanceResult * resultPtr;
  dart::collision::DistanceOption option(false, 0.0, nullptr);
  double distance = robotCollisionGroupPtr_->distance(otherCollisionGroupPtr_.get(), option, resultPtr);
  std::cout<<"The closest distance to the obstacles is: "<< distance<<std::endl;
}


void collisionAvoidanceTask::initializeCollisionGroups(
		   const dart::simulation::WorldPtr& world,
		   const dart::dynamics::SkeletonPtr & skelPtr)
{
  auto collisionEngine = world->getConstraintSolver()->getCollisionDetector();    
  robotCollisionGroupPtr_ = collisionEngine->createCollisionGroup(robotPtr_.get());
  otherCollisionGroupPtr_ = collisionEngine->createCollisionGroup(skelPtr.get());
  assert(robotCollisionGroupPtr_!= nullptr);
  assert(otherCollisionGroupPtr_!= nullptr);
  collisionGroupsInitialized_ = true;
}

void collisionAvoidanceTask::addObstacle(const dart::dynamics::SkeletonPtr & skelPtr)
{
  otherCollisionGroupPtr_->addShapeFramesOf(skelPtr.get());
}
