# ifndef MOTIONTASK_HPP
# define MOTIONTASK_HPP

# include <manipulatorTasks/metaManipulatorTask.hpp>
# include <dart/dynamics/SimpleFrame.hpp>

class motionTask: public metaManipulatorTask{

public:
  /// We use this task to track the motion of reference frame with the end-effector frame
  motionTask(const std::string &endEffectorName,
	     const dart::dynamics::SkeletonPtr& skelPtr,
	     const pt::ptree configurationDataTree,
	     double weight,
	     double Kv,
	     double Kp,
	     Eigen::Vector3d selectionV,
	     std::shared_ptr<dart::dynamics::SimpleFrame> referenceFramePtr
	     ):
    metaManipulatorTask(endEffectorName,
			referenceFramePtr->getTransform().translation(), /// By default, it returns the transform w.r.t. the world frame. 
			skelPtr,
			configurationDataTree,
			weight,
			Kv,
			Kp,
			selectionV ){
    std::cout<<"the initial translation is: "<< eePtr_->getTransform().translation()<<std::endl;
    referenceFramePtr_ = referenceFramePtr;
    updateReferences_();
  }
  void update() override;
  ~motionTask(){}
private:
  void updateReferences_(); // Update the desired position, velocity and acceleration.
  std::shared_ptr<dart::dynamics::SimpleFrame> referenceFramePtr_;// Pointer to the reference frame 
  Eigen::Vector3d velGoal_;
  Eigen::Vector3d accGoal_;
};

# endif
