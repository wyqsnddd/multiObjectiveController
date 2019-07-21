# include <collision/collisionPairVisual.hpp>

collisionPairVisual::collisionPairVisual(const dart::collision::DistanceResult * resultPtr ):  dart::gui::osg::ViewerAttachment(), resultPtr_(resultPtr)
{
  initialize();
  std::cout<<"collisionPairVisual is initialized."<<std::endl;
}


void collisionPairVisual::initialize()
{


  nodeOneFramePtr_ = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), "cNodeOne");

  double radius = 0.15;

  nodeOneFramePtr_->setShape(std::make_shared<dart::dynamics::SphereShape>(radius/4.0));
  nodeOneFramePtr_->getShape()->addDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR);
  nodeOneFramePtr_->getVisualAspect(true)->setColor(dart::Color::Red(1.0));
  /// Add the node to the viewer
  //
  nodeOnePtr_ = new dart::gui::osg::ShapeFrameNode(nodeOneFramePtr_.get(), nullptr);
  addChild(nodeOnePtr_);


  nodeTwoFramePtr_ = std::make_shared<dart::dynamics::SimpleFrame>(
        dart::dynamics::Frame::World(), "cNodeTwo");
  nodeTwoFramePtr_->setShape(std::make_shared<dart::dynamics::SphereShape>(radius/4.0));
  nodeTwoFramePtr_->getShape()->addDataVariance(dart::dynamics::Shape::DYNAMIC_COLOR);
  nodeTwoFramePtr_->getVisualAspect(true)->setColor(dart::Color::Red(1.0));
  /// Add the node to the viewer
  nodeTwoPtr_ = new dart::gui::osg::ShapeFrameNode(nodeTwoFramePtr_.get(), nullptr);
  addChild(nodeTwoPtr_);
}

void collisionPairVisual::refresh()
{
  // Get the new transform: 	
  Eigen::Isometry3d tfOne(Eigen::Isometry3d::Identity());
  tfOne.translation() = resultPtr_->nearestPoint1;
  nodeOneFramePtr_->setTransform(tfOne);
  nodeOnePtr_->refresh();
  nodeOneFramePtr_->getShape()->removeDataVariance(
          dart::dynamics::Shape::DYNAMIC_PRIMITIVE);
 // Get the new transform: 	
  Eigen::Isometry3d tfTwo(Eigen::Isometry3d::Identity());
  tfTwo.translation() = resultPtr_->nearestPoint2;
  nodeTwoFramePtr_->setTransform(tfTwo);
  nodeTwoPtr_->refresh();
  nodeTwoFramePtr_->getShape()->removeDataVariance(
          dart::dynamics::Shape::DYNAMIC_PRIMITIVE);

}
