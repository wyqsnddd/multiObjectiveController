# pragma once


#include <osg/Geode>
#include <dart/gui/osg/Viewer.hpp>
#include <dart/gui/osg/ShapeFrameNode.hpp>

#include <dart/dynamics/SmartPointer.hpp>

#include <dart/dynamics/BodyNode.hpp>
#include <dart/dynamics/SimpleFrame.hpp>
#include <dart/dynamics/SphereShape.hpp>
# include <dart/collision/CollisionGroup.hpp> 

class collisionPairVisual: public dart::gui::osg::ViewerAttachment
{
public:
  collisionPairVisual(const dart::collision::DistanceResult * resultPtr );
  inline ~collisionPairVisual(){}

  /// Update the collisionCheckingResult. 
  void refresh() override final;
  inline const dart::collision::DistanceResult * getDistanceResult()
  {
    return resultPtr_; 
  }

protected:
  /// Pointer to the DistanceResult
  const dart::collision::DistanceResult * resultPtr_; 

  /// Initialize the memory used by this visual
  void initialize();

  std::shared_ptr<dart::dynamics::SimpleFrame> nodeOneFramePtr_;
  ::osg::ref_ptr<dart::gui::osg::ShapeFrameNode> nodeOnePtr_;
  //std::shared_ptr<dart::gui::osg::ShapeFrameNode> nodeOnePtr_;
  std::shared_ptr<dart::dynamics::SimpleFrame> nodeTwoFramePtr_;
  ::osg::ref_ptr<dart::gui::osg::ShapeFrameNode> nodeTwoPtr_;
  //std::shared_ptr<dart::gui::osg::ShapeFrameNode> nodeTwoPtr_;
};
