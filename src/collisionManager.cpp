# include <collision/collisionManager.hpp>

void obstacle::addBody(const dart::dynamics::BodyNodePtr &otherLinkPtr)
{
  std::size_t originalSize = bodies.size();
  dart::collision::DistanceResult * resultPtr = new dart::collision::DistanceResult();
  collisionPairVisual * visualAspectPtr = new collisionPairVisual(resultPtr);

  assert(resultPtr!= nullptr);
  assert(visualAspectPtr!= nullptr);

  bodies[otherLinkPtr->getName()] = { resultPtr,
	  visualAspectPtr
  };

  if(++originalSize != bodies.size())
    throw std::runtime_error("addBody failed");

}

void collisionManager::addBody(const dart::dynamics::BodyNodePtr & bodyLinkPtr)
{

  auto collisionEngine = world_->getConstraintSolver()->getCollisionDetector();    
  // (1) Add to the body container
  bodyPair samplePair = {bodyLinkPtr, collisionEngine->createCollisionGroup()};
  //std::pair<const dart::dynamics::BodyNodePtr &,  std::unique_ptr< dart::collision::CollisionGroup > > samplePair(bodyLinkPtr, collisionEngine->createCollisionGroup());
  samplePair.collisionGroupPtr->addShapeFramesOf(samplePair.bodyNodePtr);

  //bodies_.push_back(samplePair);
  // bodies_.emplace_back(
  
  // (2) Go through all the obstacles
  for (auto it = obstacles_.begin(); it!=obstacles_.end();++it)  
  {
    (*it)->addBody(bodyLinkPtr);
  }
}
/*
bool  obstacleComp(const obstacle * a,  const obstacle * b) 
{ 
     return (a->resultPtr->minDistance)<(b->resultPtr->minDistance);
}
*/
void collisionManager::update()
{
  // (0) distance check through all the obstacles
  for (auto idx = bodies_.begin(); idx!=bodies_.end(); ++idx)
  {
    for (auto it = obstacles_.begin(); it!=obstacles_.end(); ++it)
    {
       (*idx).collisionGroupPtr->distance((*it)->obstacleCollisionGroupPtr.get(), *optionPtr, (*it)->bodies[(*idx).bodyNodePtr->getName()].resultPtr);
    } 
  }
/* 
  // (1) Sorting by ascending Order
  std::sort(obstacles_.begin(), obstacles_.end(), obstacleComp);
  // (2) point to the min distance pair
  minResultPtr_ = (*obstacles_.begin())->resultPtr; 
  minVisualAspectPtr_ = (*obstacles_.begin())->visualAspectPtr;
  */
}

void collisionManager::addObstacle( const dart::dynamics::BodyNodePtr &otherLinkPtr )
{
  auto collisionDetectionEngine = world_->getConstraintSolver()->getCollisionDetector();

  obstacle* newObstacle = new obstacle(collisionDetectionEngine->createCollisionGroup());
	  
	  //new obstacle(collisionDetectionEngine->createCollisionGroup());
  newObstacle->obstacleCollisionGroupPtr->addShapeFramesOf(otherLinkPtr);
  newObstacle->obstacleCollisionGroupPtr->setAutomaticUpdate(true);

	  //resultPtr,
	  //visualAspectPtr};
  // Go through the bodies: 
  for (auto it = bodies_.begin(); it!=bodies_.end(); ++it) 
  {
    dart::collision::DistanceResult * resultPtr = new dart::collision::DistanceResult();
    collisionPairVisual * visualAspectPtr =  new collisionPairVisual(resultPtr);
    newObstacle->bodies[(*it).bodyNodePtr->getName()] = {resultPtr, visualAspectPtr};
  }

  obstacles_.push_back(newObstacle);

  //minVisualAspectPtr_ = newObstacle->visualAspectPtr;
  //minResultPtr_ = newObstacle->resultPtr;
}
