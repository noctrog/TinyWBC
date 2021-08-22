#include <boost/filesystem.hpp>
#include <boost/dll.hpp>

#include <dart/dart.hpp>
#include <dart/dynamics/SmartPointer.hpp>
#include <dart/dynamics/WeldJoint.hpp>
#include <dart/dynamics/ZeroDofJoint.hpp>
#include <dart/gui/osg/osg.hpp>
#include <dart/math/Helpers.hpp>
#include <dart/utils/urdf/DartLoader.hpp>

#include <HyqWorldNode.hpp>

#include <configure.h>

using namespace dart;

dynamics::SkeletonPtr
createFloor(void)
{
  const auto floor_shape = std::make_shared<dynamics::BoxShape>(Eigen::Vector3d(5.0, 5.0, 0.2));
  const auto floor_skeleton = dynamics::Skeleton::create();
  const auto floorAndWorld
    = floor_skeleton->createJointAndBodyNodePair<dynamics::WeldJoint>();
  const auto floor_body = floorAndWorld.second;
  floor_body->createShapeNodeWith<
    dynamics::VisualAspect,
    dynamics::CollisionAspect,
    dynamics::DynamicsAspect>(floor_shape);
  Eigen::Isometry3d floor_tf(Eigen::Isometry3d::Identity());
  floor_tf.translation().z() = -0.7;
  floor_body->getParentJoint()->setTransformFromParentBodyNode(floor_tf);
  return floor_skeleton;
}

dynamics::SkeletonPtr
loadHyqRobot(void)
{
  dart::utils::DartLoader loader;

  // Retrieve the robot urdf path
  std::string pkg_path, urdf_path;
  if (boost::filesystem::exists(EXAMPLE_RESOURCE_DIR "/urdf/hyq.urdf"))
    pkg_path = EXAMPLE_RESOURCE_DIR;
  else if (boost::filesystem::exists(EXAMPLE_RESOURCE_INSTALL_DIR "/urdf/hyq.urdf"))
    pkg_path = EXAMPLE_RESOURCE_INSTALL_DIR;
  else
    urdf_path = boost::dll::program_location().parent_path().string();
  urdf_path = pkg_path + "/urdf/hyq.urdf";

  // Load robot from URDF
  loader.addPackageDirectory("hyq_urdf", pkg_path);
  dynamics::SkeletonPtr robot = loader.parseSkeleton(urdf_path);

	// Make the robot transparent
	if (robot) {
		for (const auto& node : robot->getBodyNodes())
			for (const auto& shape : node->getShapeNodes())
				if (shape->getVisualAspect())
					shape->getVisualAspect()->setAlpha(0.7);
	}

  robot->setName("hyq");

  return robot;
}

int main()
{
  const auto floor_skeleton = createFloor();
  const auto robot = loadHyqRobot();

  // Create a world and add the rigid body
  auto world = simulation::World::create();
  world->addSkeleton(robot);
  world->addSkeleton(floor_skeleton);

  // Wrap a WorldNode around it
  ::osg::ref_ptr<HyqWorldNode> node
      = new HyqWorldNode(world, robot);

  // Create a Viewer and set it up with the WorldNode
  auto viewer = gui::osg::ImGuiViewer();
  viewer.addWorldNode(node);

  viewer.addInstructionText("Press space to start free falling the box.\n");
  std::cout << viewer.getInstructions() << std::endl;

	// Add control widget
	viewer.getImGuiHandler()->addWidget(
			std::make_shared<ControllerWidget>(&viewer, world, node->getController()));

  // Set up the window to be 640x480
  viewer.setUpViewInWindow(0, 0, 640, 480);

  // Adjust the viewpoint of the Viewer
  viewer.getCameraManipulator()->setHomePosition(
      ::osg::Vec3(2.57f, 3.14f, 1.64f),
      ::osg::Vec3(0.00f, 0.00f, 0.00f),
      ::osg::Vec3(-0.24f, -0.25f, 0.94f));

  // We need to re-dirty the CameraManipulator by passing it into the viewer
  // again, so that the viewer knows to update its HomePosition setting
  viewer.setCameraManipulator(viewer.getCameraManipulator());

  // Begin running the application loop
  viewer.run();

  return 0;
}
