# ifndef LINEARVELOCITYTASK_HPP
# define LINEARVELOCITYTASK_HPP

# include <manipulatorTasks/metaManipulatorTask.hpp>

class linearVelocityTask: public metaManipulatorTask{

	public:
		linearVelocityTask(const std::string &endEffectorName, const Eigen::Vector3d & desiredVelocity, const dart::dynamics::SkeletonPtr& skelPtr, const pt::ptree configurationDataTree, double weight, double Kv, double Kp, Eigen::Vector3d selectionV):metaManipulatorTask(endEffectorName, desiredVelocity, skelPtr, configurationDataTree, weight, Kv, Kp, selectionV ){

			// std::cout<<"the initial translation is: "<< eePtr_->getTransform().translation()<<std::endl;
		}
	
		// linearVelocityTask(const std::string &endEffectorName, const Eigen::Vector3d & desiredVelocity, const dart::dynamics::SkeletonPtr& skelPtr, const pt::ptree configurationDataTree, double weight, double Kv, double Kp, Eigen::Vector3d selectionV):metaManipulatorTask(endEffectorName, desiredVelocity, skelPtr, configurationDataTree, weight, Kv, Kp, selectionV ){

		//}
		void update() override;
		~linearVelocityTask(){}

};

# endif
