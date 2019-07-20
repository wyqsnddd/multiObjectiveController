# include <collision/collisionManager.hpp>

bool  obstacleComp(const obstacle * a,  const obstacle * b) 
{ 
     return (a->resultPtr->minDistance)<(b->resultPtr->minDistance);
}

void collisionManager::update()
{
  // (0) distance check through all the obstacles
  for (auto it = obstacles_.begin(); it!=obstacles_.end(); ++it)
  {
    bodyCollisionGroupPtr_->distance((*it)->collisionGroupPtr.get(), *optionPtr, (*it)->resultPtr);
  }
  // (1) Sorting by ascending Order
  std::sort(obstacles_.begin(), obstacles_.end(), obstacleComp);
  // (2) point to the min distance pair
  minResultPtr_ = (*obstacles_.begin())->resultPtr; 
  minVisualAspectPtr_ = (*obstacles_.begin())->visualAspectPtr;
}
void collisionManager::addObstacle( const dart::dynamics::BodyNodePtr &otherLinkPtr )
{
  auto collisionDetectionEngine = bodyCollisionGroupPtr_->getCollisionDetector();
  dart::collision::DistanceResult * resultPtr = new dart::collision::DistanceResult();
  collisionPairVisual * visualAspectPtr =  new collisionPairVisual(resultPtr);

  obstacle* newObstacle = new obstacle{collisionDetectionEngine->createCollisionGroup(),
	  resultPtr,
	  visualAspectPtr};
  newObstacle->collisionGroupPtr->addShapeFramesOf(otherLinkPtr);
  newObstacle->collisionGroupPtr->setAutomaticUpdate(true);

  obstacles_.push_back(newObstacle);

  minVisualAspectPtr_ = newObstacle->visualAspectPtr;
  minResultPtr_ = newObstacle->resultPtr;
}
