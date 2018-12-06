#include <dart/dart.hpp>
#include <dart/gui/osg/osg.hpp>
# include <controllers/gravityCompensationController.hpp>
# include <controllers/manipulatorQpController.hpp>
# include <controllers/metaController.hpp>
 
//==============================================================================
class simpleWorldNode: public dart::gui::osg::WorldNode
{
	public:

		simpleWorldNode(const dart::simulation::WorldPtr& world = nullptr)
			: dart::gui::osg::WorldNode(world)
		{
			// Set up the customized WorldNode
		}

		void customPreRefresh()
		{
			// Use this function to execute custom code before each time that the
			// window is rendered. This function can be deleted if it does not need
			// to be used.
		}

		void customPostRefresh()
		{
			// Use this function to execute custom code after each time that the
			// window is rendered. This function can be deleted if it does not need
			// to be used.
		}

		void customPreStep()
		{
			// Use this function to execute custom code before each simulation time
			// step is performed. This function can be deleted if it does not need
			// to be used.
			controllerPtr_->update();	
			/*
			Eigen::VectorXd tau = controllerPtr_->computeTorques();
			std::cout<<"Before simulation, the computed torque is: "<< tau<<std::endl;
			getWorld()->getSkeleton(4)->setForces(tau);
			std::cout<<"After simulation, the robot torque is: "<< getWorld()->getSkeleton(4)->getForces()<<std::endl;
			//getWorld()->step();
			*/
		}

		void customPostStep()
		{
			// Use this function to execute custom code after each simulation time
			// step is performed. This function can be deleted if it does not need
			// to be used.
			// std::cout<<"After simulation, the robot torque is: "<< getWorld()->getSkeleton(4)->getForces()<<std::endl;
		}

		void setController(metaController* inputControllerPtr) {
			controllerPtr_ = inputControllerPtr;
		}
	private:
		//gravityCompensationController * controllerPtr_;
		//gravityCompensationController * controllerPtr_;
		metaController * controllerPtr_;

};

