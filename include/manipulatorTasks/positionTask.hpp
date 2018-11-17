# ifndef POSITIONTASK_HPP
# define POSITIONTASK_HPP

# include <manipulatorTasks/metaManipulatorTask.hpp>

class positionTask: public metaManipulatorTask{

	public:
		positionTask(const std::string &endEffectorName, const Eigen::Vector3d & desiredPosition, const dart::dynamics::SkeletonPtr& skelPtr, const pt::ptree configurationDataTree, double weight, double Kv, double Kp, Eigen::Vector3d selectionV):metaManipulatorTask(endEffectorName, desiredPosition, skelPtr, configurationDataTree, weight, Kv, Kp, selectionV ){

			std::cout<<"the initial translation is: "<< eePtr_->getTransform().translation()<<std::endl;
		}
		void update() override;
		~positionTask(){}

};

# endif
