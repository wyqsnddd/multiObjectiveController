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
#include <dart/utils/urdf/DartLoader.hpp>
#include <dart/utils/utils.hpp>

# include "simpleWorldNodeOne.hpp"
# include "simpleEventHandler.hpp"
# include "simpleWidget.hpp"

# include <controllers/gravityCompensationController.hpp>
# include <controllers/manipulatorQpController.hpp>
# include <controllers/metaController.hpp>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>


# include <collision/collisionPairVisual.hpp>
// Short alias for this namespace
namespace pt = boost::property_tree;


//==============================================================================
int main(int argc, char** argv)
{

	// Create a root
	pt::ptree robotOneConfig;
	pt::ptree robotTwoConfig;
	//
	// // Load the json file in this ptree
	pt::read_json("../config/collision_one.json", robotOneConfig);
	pt::read_json("../config/collision_one_robot_two.json", robotTwoConfig);

	// double jointUnitWeight = root.get<double>("qpController.jointUnitWeight", 0);
	// double jointUnitWeight = root.get<double>("qpController.jointUnitWeight");
	// std::cout<<"The joint unit weight is: "<< jointUnitWeight <<std::endl;

	// Create a world
	dart::simulation::WorldPtr worldPtr 
		= dart::utils::SkelParser::readWorld("dart://sample/skel/impact_wall.skel");

	assert(worldPtr != nullptr);

	 

	// Load the robot

	dart::utils::DartLoader loader;
	dart::dynamics::SkeletonPtr robot =
		loader.parseSkeleton("dart://sample/urdf/KR5/KR5_sixx_R650.urdf");

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
	worldPtr->addSkeleton(robot);
	dart::dynamics::SkeletonPtr robotTwo =
		loader.parseSkeleton("dart://sample/urdf/KR5/KR5_sixx_R650.urdf");
	worldPtr->addSkeleton(robotTwo);
	

	// Rotate and move the ground so that z is upwards
	for (unsigned i = 0; i<worldPtr->getNumSkeletons(); i++){
		//dart::dynamics::SkeletonPtr tempSkeletonPtr = worldPtr->getSkeleton("ground_skeleton");
		dart::dynamics::SkeletonPtr tempSkeletonPtr = worldPtr->getSkeleton(i);

		Eigen::Isometry3d temp_tf =
			tempSkeletonPtr->getJoint(0)->getTransformFromParentBodyNode();

		// temp_tf.pretranslate(Eigen::Vector3d(0,0,0.5));
		temp_tf.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d(1,0,0)));
		tempSkeletonPtr->getJoint(0)->setTransformFromParentBodyNode(temp_tf);

	}
	
	// Rotate the robot so that z is upwards (default transform is not Identity)
	//robot->getJoint(0)->setTransformFromParentBodyNode(Eigen::Isometry3d::Identity());

	// Read the transform and set: 
	auto robotTwoTransform = robotTwo->getJoint(0)->getTransformFromParentBodyNode();
	Eigen::Isometry3d::Identity();
	robotTwoTransform.translation().x() = 0.0; 
	robotTwoTransform.translation().y() = 1.0; 
	robotTwoTransform.translation().z() = 0.0; 
	robotTwo->getJoint(0)->setTransformFromParentBodyNode(robotTwoTransform);

	//gravityCompensationController * sampleControllerPtr = new gravityCompensationController(robot);
	manipulatorQpController* sampleQpControllerPtr = new manipulatorQpController(robot, robotOneConfig);
	manipulatorQpController* sampleQpControllerTwoPtr = new manipulatorQpController(robotTwo, robotTwoConfig);
	//metaController* sampleQpControllerPtr = new metaController(robot);
	//sampleQpControllerPtr->getTask("collisionAvoidanceTask")->initializeCollisionGroups(worldPtr, worldPtr->getSkeleton("wall"));
	// Add a target object to the world
	dart::gui::osg::InteractiveFramePtr targetPtr(
			new dart::gui::osg::InteractiveFrame(dart::dynamics::Frame::World()));

	worldPtr->addSimpleFrame(targetPtr);

	// Wrap a WorldNode around it
	osg::ref_ptr<simpleWorldNode> worldNodeOnePtr = new simpleWorldNode(worldPtr);
	
	worldNodeOnePtr->setNumStepsPerCycle(1);

	// worldNodeOnePtr->setController(sampleControllerPtr);
	//worldNodeOnePtr->setController(sampleQpControllerPtr);
	worldNodeOnePtr->addController("robotOneController", sampleQpControllerPtr);
	worldNodeOnePtr->addController("robotTwoController", sampleQpControllerTwoPtr);
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

	collisionAvoidanceTask * caTaskPtr = dynamic_cast<collisionAvoidanceTask * >(sampleQpControllerPtr->getTask("collisionAvoidanceTask").get());
	//caTaskPtr->initializeCollisionGroups(worldPtr, robot, worldPtr->getSkeleton("wall"));
	//caTaskPtr->initializeCollisionGroups(worldPtr, robot, robotTwo);
	caTaskPtr->initializeCollisionManager(worldPtr, robot->getBodyNode("shoulder"));
	caTaskPtr->addObstacle(robotTwo->getBodyNode("shoulder"));
	caTaskPtr->addObstacle(robotTwo->getBodyNode("bicep"));
	caTaskPtr->addObstacle(robotTwo->getBodyNode("elbow"));
	caTaskPtr->addObstacle(robotTwo->getBodyNode("forearm"));
	//caTaskPtr->addObstacle(robotTwo->getBodyNode("palm"));
	//caTaskPtr->addObstacle(robotTwo->getBodyNode(""));
	//caTaskPtr->initializeCollisionGroups(worldPtr, robotTwo, worldPtr->getSkeleton("wall"));
	//caTaskPtr->initializeCollisionGroups(worldPtr, worldPtr->getSkeleton("fixed_box_base"));
	//caTaskPtr->initializeCollisionGroups(worldPtr, worldPtr->getSkeleton("floating_box_skeleton"));


	if (dart::collision::CollisionDetector::getFactory()->canCreate("fcl"))
	{
		worldPtr->getConstraintSolver()->setCollisionDetector(
				dart::collision::CollisionDetector::getFactory()->create("fcl"));
	}

	std::cout<<"The collision detector is set to: "<< worldPtr->getConstraintSolver()->getCollisionDetector()->getType()<<std::endl;

	// Add the collision visualization:
	/*
	viewer.addAttachment(new collisionPairVisual(
                         caTaskPtr->getDistanceResult()));
			 */
	viewer.addAttachment(
			caTaskPtr->getCollisionManager()->getMinVisual()
			);

	// Begin running the application loop
	viewer.run();


}


