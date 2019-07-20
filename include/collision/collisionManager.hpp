# pragma once 

# include <dart/collision/CollisionGroup.hpp> 
# include <dart/constraint/ConstraintSolver.hpp> 
# include <dart/collision/CollisionDetector.hpp> 
# include <dart/simulation/World.hpp> 
# include <collision/collisionPairVisual.hpp>
# include <algorithm>
 
struct obstacle
{
 std::unique_ptr< dart::collision::CollisionGroup > collisionGroupPtr;
 dart::collision::DistanceResult * resultPtr;
 collisionPairVisual * visualAspectPtr; 
};

class collisionManager
{
  public:
   collisionManager(
		   const dart::simulation::WorldPtr & worldPtr,
		   const dart::dynamics::BodyNodePtr & bodyLinkPtr
		   ): world_(worldPtr), body_(bodyLinkPtr) 
   {
     //detector_= worldPtr->getConstraintSolver()->getCollisionDetector();
     
     auto collisionEngine = world_->getConstraintSolver()->getCollisionDetector();    
     bodyCollisionGroupPtr_ = collisionEngine->createCollisionGroup(); 
     bodyCollisionGroupPtr_->addShapeFramesOf(body_);
     optionPtr = new dart::collision::DistanceOption(true, 0.00, nullptr);
   }
   ~collisionManager()
   {
     free(optionPtr);
     for(auto it = obstacles_.begin(); it!=obstacles_.end(); ++it)
     {
       delete *it;
     }
   }

  inline const dart::collision::DistanceResult * getMinResult()
  {
    return minResultPtr_; 
  }
  inline collisionPairVisual * getMinVisual()
  {
    return minVisualAspectPtr_; 
  }
  void update();
  void addObstacle( const dart::dynamics::BodyNodePtr & otherLinkPtr);

  protected: 
  const dart::simulation::WorldPtr & world_;
  const dart::dynamics::BodyNodePtr & body_;

  collisionPairVisual * minVisualAspectPtr_; 
  dart::collision::DistanceResult * minResultPtr_;

  dart::collision::DistanceOption * optionPtr; 
  std::unique_ptr< dart::collision::CollisionGroup > bodyCollisionGroupPtr_;
  std::vector<obstacle*>  obstacles_;

};
