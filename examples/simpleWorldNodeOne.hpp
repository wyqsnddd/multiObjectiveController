/* Copyright 2018-2019 CNRS-UM LIRMM
 *
 * \author Yuquan Wang 
 *
 * 
 *
 * multiObjectiveController is free software: you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation, either version 3 of the License,
 * or (at your option) any later version.
 *
 * pyQpController is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU Lesser
 * General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with multiObjectiveController. If not, see
 * <http://www.gnu.org/licenses/>.
 */

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
			for (auto it = controllers_.begin(); it!=controllers_.end(); ++it)
			{
		 	  it->second->update();	
			}
			//controllerPtr_->update();	


			/*
			getWorld()->step();
			*/
		}

		void customPostStep()
		{
			// Use this function to execute custom code after each simulation time
			// step is performed. This function can be deleted if it does not need
			// to be used.
			// std::cout<<"After simulation, the robot torque is: "<< getWorld()->getSkeleton(4)->getForces()<<std::endl;
		}
		void addController(const std::string & name, metaController* inputControllerPtr) {
			controllers_[name] = inputControllerPtr;
		}
		/*
		void setController(metaController* inputControllerPtr) {
			controllerPtr_ = inputControllerPtr;
		}
		*/
	private:
		//gravityCompensationController * controllerPtr_;
		//gravityCompensationController * controllerPtr_;
		//metaController * controllerPtr_;
		std::map<std::string, metaController * > controllers_;

};

