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
#include <dart/io/io.hpp>
#include <dart/io/urdf/urdf.hpp>

# include "simpleWorldNodeOne.hpp"
# include "simpleEventHandler.hpp"
# include "simpleWidget.hpp"

# include <controllers/gravityCompensationController.hpp>
# include <controllers/manipulatorQpController.hpp>
# include <controllers/metaController.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

// Short alias for this namespace
namespace pt = boost::property_tree;


//==============================================================================
int main(int argc, char** argv)
{

	// Create a root
	pt::ptree root;
	//
	// // Load the json file in this ptree
	pt::read_json("../config/impact_one.json", root);

	// double jointUnitWeight = root.get<double>("qpController.jointUnitWeight", 0);
	// double jointUnitWeight = root.get<double>("qpController.jointUnitWeight");
	// std::cout<<"The joint unit weight is: "<< jointUnitWeight <<std::endl;

	// Create a world
	dart::simulation::WorldPtr worldPtr 
		= dart::io::SkelParser::readWorld("dart://sample/skel/impact_wall.skel");

	assert(worldPtr != nullptr);
	// Rotate and move the ground so that z is upwards
	for (int i = 0; i<worldPtr->getNumSkeletons(); i++){
		//dart::dynamics::SkeletonPtr tempSkeletonPtr = worldPtr->getSkeleton("ground_skeleton");
		dart::dynamics::SkeletonPtr tempSkeletonPtr = worldPtr->getSkeleton(i);

		Eigen::Isometry3d temp_tf =
			tempSkeletonPtr->getJoint(0)->getTransformFromParentBodyNode();

		// temp_tf.pretranslate(Eigen::Vector3d(0,0,0.5));
		temp_tf.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(1,0,0)));
		tempSkeletonPtr->getJoint(0)->setTransformFromParentBodyNode(temp_tf);

	} 

	// Load the robot

	dart::io::DartLoader loader;

	dart::dynamics::SkeletonPtr robot =
		loader.parseSkeleton("dart://sample/urdf/KR5/KR5_sixx_R650.urdf");
	worldPtr->addSkeleton(robot);
	// Set the colors of the models to obey the shape's color specification
	for(std::size_t i=0; i<robot->getNumBodyNodes(); ++i)
	{
		dart::dynamics::BodyNode* bn = robot->getBodyNode(i);
		auto shapeNodes = bn->getShapeNodesWith<dart::dynamics::VisualAspect>();
		for(auto shapeNode : shapeNodes)
		{
			std::shared_ptr<dart::dynamics::MeshShape> mesh =
				std::dynamic_pointer_cast<dart::dynamics::MeshShape>(shapeNode->getShape());
			if(mesh)
				mesh->setColorMode(dart::dynamics::MeshShape::SHAPE_COLOR);
		}
	}

	// Rotate the robot so that z is upwards (default transform is not Identity)
	robot->getJoint(0)->setTransformFromParentBodyNode(Eigen::Isometry3d::Identity());

	//gravityCompensationController * sampleControllerPtr = new gravityCompensationController(robot);
	manipulatorQpController* sampleQpControllerPtr = new manipulatorQpController(robot, root);
	//metaController* sampleQpControllerPtr = new metaController(robot);

	// Add a target object to the world
	dart::gui::osg::InteractiveFramePtr targetPtr(
			new dart::gui::osg::InteractiveFrame(dart::dynamics::Frame::World()));

	worldPtr->addSimpleFrame(targetPtr);

	// Wrap a WorldNode around it
	osg::ref_ptr<simpleWorldNode> worldNodeOnePtr = new simpleWorldNode(worldPtr);
	
	worldNodeOnePtr->setNumStepsPerCycle(10);

	// worldNodeOnePtr->setController(sampleControllerPtr);
	worldNodeOnePtr->setController(sampleQpControllerPtr);
	// Create a Viewer and set it up with the WorldNode
	dart::gui::osg::ImGuiViewer viewer;
	viewer.addWorldNode(worldNodeOnePtr);
	// Add widget 
	viewer.getImGuiHandler()->addWidget(
			std::make_shared<simpleWidget>(&viewer, worldPtr));


	// Active the drag-and-drop feature for the target
	viewer.enableDragAndDrop(targetPtr.get());

	// Pass in the custom event handler
	viewer.addEventHandler(new simpleEventHandler);

	// Print out instructions
	std::cout << viewer.getInstructions() << std::endl;
 

	// Set up the window to be 640x480
	viewer.setUpViewInWindow(0, 0, 640, 480);

	// Adjust the viewpoint of the Viewer
	viewer.getCameraManipulator()->setHomePosition(::osg::Vec3( -0.08,  2.69, 1.26),
			::osg::Vec3( 0.15,  1.76, 0.98),
			::osg::Vec3(0.02, -0.28, 0.96));

	// We need to re-dirty the CameraManipulator by passing it into the viewer
	// again, so that the viewer knows to update its HomePosition setting
	viewer.setCameraManipulator(viewer.getCameraManipulator());

	// Begin running the application loop
	viewer.run();
}


