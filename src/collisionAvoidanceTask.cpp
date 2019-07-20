# include <manipulatorTasks/collisionAvoidanceTask.hpp>


void collisionAvoidanceTask::update()
{
  if(!initialized())
    throw std::runtime_error("collisionAvoidanceTask: the CollisionGroups have not yet initialized. ");
  /*
  dart::collision::DistanceOption option(true, 0.00, nullptr);
  double distance = robotCollisionGroupPtr_->distance(otherCollisionGroupPtr_.get(), option, resultPtr_);
  //double distance = robotCollisionGroupPtr_->distance(option, resultPtr_);
  //double distance = otherCollisionGroupPtr_->distance(robotCollisionGroupPtr_.get(), option, resultPtr_) ;
  std::cout<<"The closest distance to the obstacles is: "<< distance<<std::endl;
  if(getDistanceResult()->found())
  {
   std::cout<<"The shape frame 1 is: "<< getDistanceResult()->shapeFrame1->getName()<<std::endl;
   std::cout<<"The shape frame 2 is: "<< getDistanceResult()->shapeFrame2->getName()<<std::endl;

  }
  */ 
  collisionManagerPtr_->update();

}

void collisionAvoidanceTask::initializeCollisionManager(const dart::simulation::WorldPtr& world,
		   const dart::dynamics::BodyNodePtr & bodyNode)
{
  collisionManagerPtr_.reset( new collisionManager(world, bodyNode));

  collisionGroupsInitialized_ = true;
}


void collisionAvoidanceTask::initializeCollisionGroups(
		   const dart::simulation::WorldPtr& world,
		   const dart::dynamics::SkeletonPtr & robot,
		   const dart::dynamics::SkeletonPtr & obstacle)
{
  auto collisionEngine = world->getConstraintSolver()->getCollisionDetector();    
  //auto collisionEngine = dart::collision::CollisionDetector::getFactory()->create("fcl");
  //robotCollisionGroupPtr_ = collisionEngine->createCollisionGroup(robot.get());
  robotCollisionGroupPtr_ = collisionEngine->createCollisionGroup();
  robotCollisionGroupPtr_->addShapeFramesOf(robot->getBodyNode("palm"));
  //robotCollisionGroupPtr_->addShapeFramesOf(robot->getBodyNode("forearm"));
  //robotCollisionGroupPtr_->addShapeFramesOf(robot->getBodyNode("bicep"));
  //robotCollisionGroupPtr_->addShapeFramesOf(robot->getBodyNode("elbow"));
  //robotCollisionGroupPtr_ = collisionEngine->createCollisionGroup(world->getSkeleton("floating_box_skeleton").get());
 // otherCollisionGroupPtr_ = collisionEngine->createCollisionGroup(obstacle.get());
  otherCollisionGroupPtr_ = collisionEngine->createCollisionGroup();
  //otherCollisionGroupPtr_->addShapeFramesOf(obstacle->getBodyNode("palm"));
  otherCollisionGroupPtr_->subscribeTo(world->getSkeleton("wall"));
  otherCollisionGroupPtr_->subscribeTo(world->getSkeleton("floating_box_skeleton"));
  //otherCollisionGroupPtr_->addShapeFramesOf(obstacle->getBodyNode("elbow"));
  //otherCollisionGroupPtr_->addShapeFramesOf(obstacle->getBodyNode("bicep"));
  //otherCollisionGroupPtr_->addShapeFramesOf(obstacle->getBodyNode("forearm"));
  //otherCollisionGroupPtr_->subscribeTo(obstacle);
  //otherCollisionGroupPtr_->addShapeFramesOf(obstacle);
  //otherCollisionGroupPtr_->addShapeFramesOf(obstacle->getBodyNode("shoulder"));
 //robotCollisionGroupPtr_->subscribeTo(obstacle);
 robotCollisionGroupPtr_->setAutomaticUpdate(true);
 otherCollisionGroupPtr_->setAutomaticUpdate(true);
  assert(robotCollisionGroupPtr_!= nullptr);
  assert(otherCollisionGroupPtr_!= nullptr);
  collisionGroupsInitialized_ = true;

  //robotCollisionGroupPtr_->getShapeFrame(0)->getCollisionAspect()->


  //robotCollisionGroupPtr_->removeShapeFrame(robotCollisionGroupPtr_->getShapeFrame(0));
  //otherCollisionGroupPtr_->removeShapeFrame(otherCollisionGroupPtr_->getShapeFrame(0));
  int sn = robotCollisionGroupPtr_->getNumShapeFrames();
  std::cout<<"the number of shape frames of robot is: "<<sn<<std::endl;
  for(int ii = 0; ii<sn; ++ii)
  {
	  std::cout<<robotCollisionGroupPtr_->getShapeFrame(ii)->getShape()->getType()<<"."<<std::endl;
	  /*
	  robotCollisionGroupPtr_->getShapeFrame(ii)->setShape(std::make_shared<dart::dynamics::BoxShape>(robotCollisionGroupPtr_->getShapeFrame(ii)->getShape()->getBoundingBox().computeFullExtents()));
	  */
  //nodeOneFramePtr_->setShape(std::make_shared<dart::dynamics::SphereShape>(radius/4.0));

  }



}
void collisionAvoidanceTask::addObstacle(const dart::dynamics::BodyNodePtr& BodyNodePtr)
{
  collisionManagerPtr_->addObstacle(BodyNodePtr);
}
/*
void collisionAvoidanceTask::addObstacle(const dart::dynamics::SkeletonPtr & skelPtr)
{
  otherCollisionGroupPtr_->addShapeFramesOf(skelPtr.get());
}
*/
