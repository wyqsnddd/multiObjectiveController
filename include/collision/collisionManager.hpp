# pragma once 

# include <dart/collision/CollisionGroup.hpp> 
# include <dart/constraint/ConstraintSolver.hpp> 
# include <dart/collision/CollisionDetector.hpp> 
# include <dart/simulation/World.hpp> 
# include <collision/collisionPairVisual.hpp>
# include <algorithm>

struct attachedBody
{
 dart::collision::DistanceResult * resultPtr;
 collisionPairVisual * visualAspectPtr; 
};
struct obstacle
{
 obstacle(std::unique_ptr< dart::collision::CollisionGroup > cgPtr): obstacleCollisionGroupPtr(cgPtr)
 {
 };
 ~obstacle(){}
 std::unique_ptr< dart::collision::CollisionGroup > & obstacleCollisionGroupPtr;
 std::map<std::string, attachedBody>bodies;
 void addBody(const dart::dynamics::BodyNodePtr &otherLinkPtr);
};
struct bodyPair
{
const dart::dynamics::BodyNodePtr & bodyNodePtr;
std::unique_ptr< dart::collision::CollisionGroup > collisionGroupPtr;
};
class collisionManager
{
  public:
   collisionManager(
		   const dart::simulation::WorldPtr & worldPtr,
		   const dart::dynamics::BodyNodePtr & bodyLinkPtr
		   ): world_(worldPtr)
   {
     //detector_= worldPtr->getConstraintSolver()->getCollisionDetector();
     
     //std::unique_ptr< dart::collision::CollisionGroup > bodyCollisionGroupPtr = collisionEngine->createCollisionGroup(); 
     //bodyCollisionGroupPtr->addShapeFramesOf(bodyLinkPtr);
     addBody(bodyLinkPtr);
     optionPtr = new dart::collision::DistanceOption(true, 0.00, nullptr);
   }
   void addBody(const dart::dynamics::BodyNodePtr & bodyLinkPtr);
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
  /*
  std::vector<
	 std::pair<const dart::dynamics::BodyNodePtr &,  std::unique_ptr< dart::collision::CollisionGroup >  > 
	  > bodies_;
	  */
  std::vector<bodyPair> bodies_;
   //bodyCollisionGroupPtr_;

  collisionPairVisual * minVisualAspectPtr_; 
  dart::collision::DistanceResult * minResultPtr_;

  dart::collision::DistanceOption * optionPtr; 
  std::vector<obstacle*>  obstacles_;

};
